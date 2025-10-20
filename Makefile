.DEFAULT_GOAL := help

.PHONY: help
help: ## Display available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-18s %s\n", $$1, $$2}'

.PHONY: build
build: ## Build with release-with-debug profile
	. external/autoware/install/setup.sh && \
	cargo build --profile release-with-debug

.PHONY: run
run: ## Run the bridge
	. external/autoware/install/setup.sh && \
	cargo run --profile release-with-debug

.PHONY: clean
clean: ## Clean build artifacts
	cargo clean

.PHONY: format
format: ## Format code with rustfmt
	cargo +nightly fmt

.PHONY: lint
lint: ## Run format check and clippy
	cargo +nightly fmt --check
	. external/autoware/install/setup.sh && \
	cargo clippy --all-targets --all-features

.PHONY: test
test: ## Run tests
	. external/autoware/install/setup.sh && \
	cargo nextest run --no-fail-fast --cargo-profile release-with-debug

.PHONY: agent-setup
agent-setup: ## Setup carla_agent environment
	. external/autoware/install/setup.sh && \
	cd carla_agent && uv sync

.PHONY: agent-spawn
agent-spawn: ## Spawn test vehicles (requires CARLA running)
	. external/autoware/install/setup.sh && \
	cd carla_agent && uv run python simple_spawn.py
