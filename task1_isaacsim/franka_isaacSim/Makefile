# ──────────────────────────────────────────────────────────────────────────────
# franka_isaacSim – developer task runner
# ──────────────────────────────────────────────────────────────────────────────

COMPOSE        := docker compose
REAL_DOMAIN_ID ?= 100

.PHONY: help follower-up follower-down digital-twin-up digital-twin-down digital-twin-status \
	stack-up stack-down stack-restart

help:
	@echo "Usage: make <target>"
	@echo ""
	@echo "  follower-up                Start the cross-RMW real robot bridge (follower profile)"
	@echo "  follower-down              Stop the real robot bridge"
	@echo "  digital-twin-up            Start the complete digital twin application (sim + bridge)"
	@echo "  digital-twin-down          Stop the digital twin application"
	@echo "  digital-twin-status        Check digital twin status and verify message flow"
	@echo "  stack-up                   Start the simulation stack containers (isaac-sim + ros services)"
	@echo "  stack-down                 Stop the simulation stack"
	@echo "  stack-restart              Force-recreate containers and launch Isaac Sim with default scene"
	@echo ""
	@echo "  Default launch:             bash scripts/run_native_stream.sh --asset-name ai_cell_handBoxing"

## ── Real robot bridge (cross-RMW follower) ──────────────────────────────────
follower-up:
	$(COMPOSE) --profile follower up -d real_to_sim_bridge

follower-down:
	$(COMPOSE) --profile follower down real_to_sim_bridge

## ── Digital Twin Application ────────────────────────────────────────────────
digital-twin-up:
	@echo "=========================================="
	@echo "Starting Digital Twin Application"
	@echo "=========================================="
	@echo "Launching real-to-virtual twin setup..."
	@bash scripts/run_real_to_virtual_twin.sh
	@echo "Setting default control mode to digital_twin..."
	@ok=0; \
	for i in $$(seq 1 20); do \
		if curl -s -X POST http://localhost:8090/api/control_mode \
			-H 'Content-Type: application/json' \
			-d '{"mode":"digital_twin"}' >/dev/null 2>&1; then \
			ok=1; \
			break; \
		fi; \
		sleep 1; \
	done; \
	if [ "$$ok" -eq 1 ]; then \
		echo "  Default mode set: digital_twin"; \
	else \
		echo "  Could not set default mode automatically (controller may still be starting)"; \
	fi
	@echo ""
	@echo "=========================================="
	@echo "Digital Twin Application Started"
	@echo "=========================================="
	@echo ""
	@echo "Verify with: make digital-twin-status"
	@echo "Browser UI:  http://localhost:8090"
	@echo ""
	@echo "To stop:     make digital-twin-down"

digital-twin-down:
	@echo "Stopping Digital Twin Application..."
	@$(COMPOSE) --profile follower down real_to_sim_bridge
	@$(COMPOSE) down
	@echo "Digital Twin Application stopped"

digital-twin-status:
	@echo "=========================================="
	@echo "Digital Twin Status Check"
	@echo "=========================================="
	@echo ""
	@echo "1. Container Status:"
	@running_services="$$($(COMPOSE) ps --status running --services 2>/dev/null)"; \
	found=0; \
	for svc in browser_controller isaac-sim real_to_sim_bridge ros_republisher; do \
		if echo "$$running_services" | grep -qx "$$svc"; then \
			echo "  $$svc - running"; \
			found=1; \
		fi; \
	done; \
	if [ "$$found" -eq 0 ]; then \
		echo "  No core containers running"; \
	fi
	@echo ""
	@echo "2. Control Mode:"
	@if curl -s http://localhost:8090/api/control_mode 2>/dev/null > /dev/null; then \
		echo "  Browser controller responding"; \
	else \
		echo "  Browser controller not responding"; \
	fi
	@echo ""
	@echo "3. Bridge Status:"
	@if $(COMPOSE) ps | grep -q "real_to_sim_bridge"; then \
		echo "  Real-to-sim bridge is running"; \
	else \
		echo "  Real-to-sim bridge not running"; \
	fi
	@if $(COMPOSE) ps | grep -q "ros_republisher"; then \
		echo "  ROS republisher is running"; \
	else \
		echo "  ROS republisher not running"; \
	fi
	@echo ""
	@echo "4. Topic Pipeline Health:"
	@$(COMPOSE) exec -T real_to_sim_bridge timeout 4 bash -lc "export ROS_DOMAIN_ID=$(REAL_DOMAIN_ID); export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source /opt/ros/jazzy/setup.bash; ros2 topic list 2>/dev/null" > /tmp/real_domain_topics.txt 2>/dev/null || true; \
	inputs_ok=1; \
	for t in \
		/left/franka_robot_state_broadcaster/measured_joint_states \
		/right/franka_robot_state_broadcaster/measured_joint_states \
		/gripper/left/joint_states \
		/gripper/right/joint_states; do \
		if grep -qx "$$t" /tmp/real_domain_topics.txt 2>/dev/null; then \
			echo "  input: $$t"; \
		else \
			inputs_ok=0; \
		fi; \
	done; \
	$(COMPOSE) exec -T ros_republisher timeout 4 bash -lc "source /opt/ros/jazzy/setup.bash; ros2 topic list 2>/dev/null" > /tmp/sim_domain_topics.txt 2>/dev/null || true; \
	outputs_ok=1; \
	for t in \
		/bridge/left_joint_commands \
		/bridge/right_joint_commands \
		/bridge/left_robotiq_joint_commands \
		/bridge/right_robotiq_joint_commands; do \
		if grep -qx "$$t" /tmp/sim_domain_topics.txt 2>/dev/null; then \
			echo "  output: $$t"; \
		else \
			outputs_ok=0; \
		fi; \
	done; \
	if [ "$$inputs_ok" -eq 1 ]; then \
		echo "  Real-domain inputs visible on domain $(REAL_DOMAIN_ID)"; \
	elif [ "$$outputs_ok" -eq 1 ]; then \
		echo "  Bridge pipeline active (outputs visible on sim domain; input discovery may be limited)"; \
	else \
		echo "  Neither expected real-domain inputs nor bridge outputs are fully visible"; \
	fi
	@echo ""
	@echo "=========================================="
	@echo "Expected: ~1000 Hz on arm topics"
	@echo "If status shows issues, check logs with:"
	@echo "  docker compose logs real_to_sim_bridge"
	@echo "  docker compose logs isaac-sim"
	@echo "=========================================="

## ── Stack management ────────────────────────────────────────────────────────
stack-up:
	$(COMPOSE) up -d

stack-down:
	$(COMPOSE) --profile position down --remove-orphans

stack-restart:
	bash scripts/run_native_stream.sh --force-recreate
