.DEFAULT_GOAL := help

.PHONY: help
help: ## Display available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-18s %s\n", $$1, $$2}'

.PHONY: build
build: ## Build with release-with-debug profile
	cargo build --profile release-with-debug

.PHONY: run
run: ## Run the bridge
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
	cargo clippy --all-targets --all-features

.PHONY: test
test: ## Run tests
	cargo nextest run --no-fail-fast --cargo-profile release-with-debug

.PHONY: agent-setup
agent-setup: ## Setup carla_agent environment
	cd carla_agent && uv sync

.PHONY: agent-spawn
agent-spawn: ## Spawn test vehicles (requires CARLA running)
	cd carla_agent && uv run python simple_spawn.py
