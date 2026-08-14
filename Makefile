.PHONY: install test run

install:
	@command -v uv >/dev/null 2>&1 || { echo "Installing uv..."; curl -LsSf https://astral.sh/uv/install.sh | sh; }
	@test -d .venv || uv venv
	@echo "Installing dependencies from requirements.txt..."
	uv pip install -r requirements.txt

# Local server without Onshape/Google sign-in (upload DXF files by hand).
# Drop a PenguinCAM-config.yaml next to the app (or set PENGUINCAM_CONFIG=path)
# to use your team's machine settings; edits are picked up without a restart.
run: install
	AUTH_ENABLED=false ONSHAPE_AUTH_REQUIRED=false uv run python frc_cam_gui_app.py

test:
	@echo "Running unit tests..."
	@uv run python -m unittest discover -s tests --buffer
	@echo ""
	@echo "Running system tests..."
	@uv run python gcode_test.py --quiet
