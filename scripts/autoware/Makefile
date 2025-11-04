.PHONY: help
help: ## Display available targets
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-20s %s\n", $$1, $$2}'

.PHONY: install-deps
install-deps: ## Install play-launch tool for launching Autoware
	@echo "Installing play-launch..."
	@if command -v play_launch >/dev/null 2>&1; then \
		echo "play-launch is already installed"; \
	else \
		wget -q https://github.com/NEWSLabNTU/play-launch/releases/download/v0.2.0/play-launch_0.2.0_amd64.deb -O /tmp/play-launch_0.2.0_amd64.deb && \
		sudo dpkg -i /tmp/play-launch_0.2.0_amd64.deb && \
		rm /tmp/play-launch_0.2.0_amd64.deb && \
		echo "play-launch installed successfully"; \
	fi

.PHONY: launch
launch: ## Launch Autoware planning simulator with sample vehicle and sensor kit
	cd autoware_repo && \
	. install/setup.sh && \
	play_launch launch \
	  autoware_launch planning_simulator.launch.xml \
	  map_path:=$$HOME/autoware_map/sample-map-planning \
	  vehicle_model:=sample_vehicle \
	  sensor_model:=sample_sensor_kit
