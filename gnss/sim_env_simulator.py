from __future__ import annotations

# Backward-compatible entry name requested for simulation runs.
# Preferred usage from the project parent directory:
#   python -m rov_control.sim_env_simulator --dvl
try:
    from rov_control.real_env_simulator import main
except ModuleNotFoundError:
    # Allow running this file directly from inside the rov_control directory.
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from rov_control.real_env_simulator import main


if __name__ == "__main__":
    main()
