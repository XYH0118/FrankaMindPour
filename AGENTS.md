# Repository Guidelines

## Project Structure & Module Organization

`franka_mind_pour/` is a ROS 1 Noetic catkin package targeting Ubuntu 20.04. Its `src/` and `include/franka_mind_pour/` contain the C++ joint controller; `scripts/` contains Python ROS nodes and the bag recorder. ROS messages live in `msg/`, launch files in `launch/`, and parameters in `config/mind_pour.yaml`. Root-level `scripts/` contains build, session, and Windows-compatible UDP sender tools. See `README.md` and `CLAUDE.md` for architecture and operational details.

## Build, Test, and Development Commands

Run these from the repository root on the configured Ubuntu ROS host:

- `./scripts/compile_package.sh`: copies the package into `~/workspace/catkin_ws_fmp/` and builds with catkin. It clears that workspace and updates shell startup files. Rebuild after code, configuration, or launch changes; runtime uses the copy.
- `./scripts/run_session.sh --help`: lists accepted session options.
- `./scripts/run_session.sh stage:=udp`: starts the bridge and rate monitor without robot motion.
- `python3 scripts/fmp_udp_sender.py --host 127.0.0.1`: sends a local test pattern from another terminal.

## Coding Style & Naming Conventions

Use C++17 and `.clang-format`: Google style, two-space indentation, 100-column limit. Format changed C++ files with `clang-format -i <file>`. Follow existing camelCase methods, PascalCase classes, and trailing-underscore members. Python uses four-space indentation and snake_case names; no Python formatter or linter is configured.

Keep parameter defaults synchronized across YAML, C++ members, and README documentation. Document units; user-facing angles use `_deg`, while internal positions use radians. Register new messages and Python nodes in `CMakeLists.txt`, and update the session script whitelist when adding launch arguments.

## Testing Guidelines

There is no checked-in test suite, testing framework, CI, or coverage threshold. Verify a clean catkin build and the localhost UDP path; record commands and observations in the PR. Check command topics and packet counters. Controller changes remain unverified on hardware until an operator tests them. The previously used fake position interface is not included.

## Commit & Pull Request Guidelines

Git history is unavailable in this checkout. Use concise, imperative commit subjects describing the affected behavior. PRs should explain the change, link relevant issues, list validation performed, and identify parameter or robot-motion effects.

## Robot Safety & Real-Time Constraints

Agents must not run `stage:=move`, `stage:=rotate`, or full sessions: these move the arm. Preserve joint limits and acceleration bounds. Keep the 1 kHz controller path free of blocking operations and allocations; age commands using host receive time.
