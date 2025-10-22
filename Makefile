.DEFAULT_GOAL := help

.PHONY: help
help: ## Display available targets
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'

.PHONY: install-deps
install-deps: ## Install colcon plugins and dependencies
	bash scripts/install_deps.sh

.PHONY: build-ros2-rust
build-ros2-rust: ## Build ros2_rust packages (Rust generator, runtime, and rclrs)
	@echo "Stage 1: Building ros2_rust packages (Rust generator, runtime, and rclrs)..."
	. src/external/autoware/install/setup.sh && \
	colcon build --symlink-install --base-paths src/ros2_rust

.PHONY: build-interface
build-interface: ## Build message packages (generates Rust crates)
	@echo "Stage 2: Building message packages (generates Rust crates)..."
	. src/external/autoware/install/setup.sh && \
	. install/setup.sh && \
	colcon build --symlink-install --base-paths src/interface

.PHONY: build-packages
build-packages: ## Build autoware_carla_bridge package
	@echo "Stage 3: Building autoware_carla_bridge..."
	. src/external/autoware/install/setup.sh && \
	. install/setup.sh && \
	colcon build --symlink-install --packages-up-to autoware_carla_bridge \
	  --cargo-args --release

.PHONY: build
build: build-ros2-rust build-interface build-packages ## Build all stages (complete build)

.PHONY: launch
launch: ## Launch the bridge with ros2 launch
	. install/setup.sh && \
	ros2 launch autoware_carla_bridge autoware_carla_bridge.launch.xml

.PHONY: run
run: ## Run the bridge executable directly
	. install/setup.sh && \
	ros2 run autoware_carla_bridge autoware_carla_bridge

.PHONY: clean
clean: ## Clean build artifacts
	rm -rf build install log .cargo

.PHONY: format
format: ## Format code with rustfmt
	cargo +nightly fmt --manifest-path src/autoware_carla_bridge/Cargo.toml

.PHONY: lint
lint: ## Run format check and clippy
	cargo +nightly fmt --check --manifest-path src/autoware_carla_bridge/Cargo.toml
	. src/external/autoware/install/setup.sh && \
	. install/setup.sh && \
	cargo clippy --manifest-path src/autoware_carla_bridge/Cargo.toml

.PHONY: test
test: ## Run tests
	. src/external/autoware/install/setup.sh && \
	. install/setup.sh && \
	cargo nextest run --no-fail-fast --manifest-path src/autoware_carla_bridge/Cargo.toml

.PHONY: agent-setup
agent-setup: ## Setup carla_agent environment
	cd carla_agent && uv sync

.PHONY: agent-spawn
agent-spawn: ## Spawn test vehicles (requires CARLA running)
	cd carla_agent && uv run python simple_spawn.py
