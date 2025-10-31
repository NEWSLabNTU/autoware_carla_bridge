.DEFAULT_GOAL := help
COLCON_BUILD_FLAGS := --symlink-install --cargo-args --release

.PHONY: help
help: ## Display available targets
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'

.PHONY: install-deps
install-deps: ## Install colcon plugins and dependencies
	./scripts/install_deps.sh

.PHONY: build-ros2-rust
build-ros2-rust: ## Build ros2_rust packages (Rust generator, runtime, and rclrs)
	cd src/ros2_rust && \
	colcon build $(COLCON_BUILD_FLAGS)

.PHONY: build-interface
build-interface: ## Build message packages (generates Rust crates)
	. src/ros2_rust/install/setup.sh && \
	cd src/interface && \
	colcon build $(COLCON_BUILD_FLAGS)

.PHONY: build-packages
build-bridge: ## Build autoware_carla_bridge package
	. src/interface/install/setup.sh && \
	cd src/autoware_carla_bridge && \
	colcon build $(COLCON_BUILD_FLAGS)

.PHONY: build
build: build-ros2-rust build-interface build-bridge ## Build all stages (complete build)

.PHONY: launch
launch: ## Launch the bridge with ros2 launch
	source src/autoware_carla_bridge/install/setup.sh && \
	ros2 launch autoware_carla_bridge autoware_carla_bridge.launch.xml

.PHONY: clean-ros2-rust
clean-ros2-rust:
	cd src/ros2_rust && \
	rm -rf build install log

.PHONY: clean-interface
clean-interface:
	cd src/interface && \
	rm -rf build install log

.PHONY: clean-bridge
clean-bridge:
	cd src/autoware_carla_bridge && \
	rm -rf build install log

.PHONY: clean
clean: clean-ros2-rust clean-interface clean-bridge ## Clean build artifacts

.PHONY: format
format: ## Format code with rustfmt
	cargo +nightly fmt --manifest-path src/autoware_carla_bridge/Cargo.toml

.PHONY: lint
lint: ## Run format check and clippy
	cargo +nightly fmt --check --manifest-path src/autoware_carla_bridge/Cargo.toml
	source src/interface/install/setup.sh && \
	cargo clippy --manifest-path src/autoware_carla_bridge/Cargo.toml

.PHONY: test
test: ## Run tests
	source src/interface/install/setup.sh && \
	cargo nextest run --no-fail-fast --manifest-path src/autoware_carla_bridge/Cargo.toml

.PHONY: agent-setup
agent-setup: ## Setup carla_agent environment
	cd carla_agent && uv sync

.PHONY: agent-spawn
agent-spawn: ## Spawn test vehicles (requires CARLA running)
	cd carla_agent && uv run python simple_spawn.py
