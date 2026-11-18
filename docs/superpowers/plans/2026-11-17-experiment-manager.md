# Experiment Manager Implementation Plan

> **For agentic workers:** Execute inline using superpowers:executing-plans. Use test-driven-development for each behavior; do not launch the robot.

**Goal:** Run successive remotely named pouring sessions with reliable lifecycle notifications and Windows keyboard testing.

**Architecture:** A standalone TCP supervisor launches the existing shell runner. Private filesystem markers connect ROS readiness/completion to the supervisor without a persistent ROS master. Windows uses TCP for lifecycle and existing UDP datagrams for motion.

**Tech Stack:** Python 3.8+ standard library, Bash, ROS 1 Noetic. No new runtime dependencies.

**Spec:** ../specs/2026-11-17-experiment-manager-design.md

## Global Constraints

- TCP 5006; UDP 5005; ASCII newline framing; existing UDP payload unchanged.
- No robot execution, C++ motion changes, or AGENTS.md modifications.
- No Git metadata exists; skip worktree creation and commits.
- Three Windows destination files require escalation after local validation.

## Task 1: Supervisor protocol and lifecycle

Files: create scripts/experiment_manager.py, tests/test_experiment_manager.py,
and tests/fixtures/fake_session.py.

- [x] Write unittest cases using real temporary status directories, fake child
  subprocesses and sockets: assert STARTING then READY only after both readiness
  markers; identical START does not spawn again; changed ID metadata errors;
  BUSY on competing START; completed+zero exit gives DONE; early/nonzero exits
  give ERROR; timeout cleans child before accepting the next request; finish
  waits for the active trial; fragmented/coalesced frames and reconnect work.
- [x] Run `python -m unittest discover -s tests -v`; confirm missing supervisor
  behavior fails before implementation.
- [x] Implement SessionProcess lifecycle, ExperimentManager request ledger and
  TCP serve loop. Use argument arrays, private TemporaryDirectory, bounded
  framing and cleanup. Expose runner injection to exercise real fake children.
- [x] Rerun the suite and resolve failed contracts.

## Task 2: ROS lifecycle signals and shell exit code

Files: modify franka_mind_pour/scripts/session_manager.py,
franka_mind_pour/scripts/udp_bridge.py, scripts/run_session.sh; add
tests/test_session_lifecycle.py with ROS boundary stubs.

- [x] Add failing behavioral tests: bound bridge announces readiness and drops
  packets before accept_commands; controller readiness requires WAITING;
  completion is emitted only after result writing and recording delay.
- [x] Implement the environment-variable marker handshake; leave standalone
  behavior intact. Preserve the shell child's status with `FMP_EXIT_CODE=$?`
  immediately following wait and `exit "$FMP_EXIT_CODE"` after log cleanup.
- [x] Run lifecycle tests; check Python compilation and Bash syntax where a
  Bash runtime is available. Document unavailable ROS/hardware verification.

## Task 3: Windows managed sender and keyboard entry point

Files: modify scripts/fmp_udp_sender.py; create scripts/fmp_udp_keyboard.py and scripts/fmp_udp_keyboard.bat,
tests/test_experiment_client.py.

- [x] Write failing loopback tests for TCP line buffering, START/STATUS/finish,
  no UDP before READY, UDP payload compatibility, and invalid pattern inputs.
- [x] Implement a standalone ExperimentClient in the sender file. Integrate
  repeated ID prompts, managed pattern sending and Windows key polling using
  GetAsyncKeyState. Preserve the CommandSender API and add --udp-only.
- [x] Keep the batch target fmp_udp_keyboard.py, using the shared managed client.
- [x] Run all tests and CLI --help, then a real fake-child loopback experiment.

## Task 4: Documentation, review, deployment

Files: update README.md and CLAUDE.md; copy the sender and batch file to the
three user-specified parent paths (including the subsequently supplied keyboard Python file).

- [x] Document Ubuntu startup, ports, messages, keyboard controls, failure and
  reconnect semantics, and the absence of hardware validation.
- [x] Review lifecycle races, subprocess cleanup, exit status, and TCP framing;
  rerun tests after fixes. Never execute move/rotate/full on real ROS.
- [x] Request a narrowly scoped elevated Copy-Item command for the three
  validated Windows files; verify source/destination SHA256 hashes afterward.

## Verification outcome (2026-11-18)

- 23 unittest cases: 22 passed, 1 Linux-only detached-process integration test
  skipped on Windows. Loopback tests exercise TCP/UDP and fake child processes.
- Bash syntax check passed; `bash tests/check_shell_exit.sh` passed with fake
  ROS/environment/logging commands, verifying child exit status preservation.
- Python compilation and Python 3.8 syntax checks passed. Deployed Windows CLI
  help works; all three destination SHA256 hashes match the tested sources.
- Independent read-only review completed; four findings fixed with regressions.
- No ROS build, actual ROS launch, physical keyboard/foreground-window manual
  test, or robot execution performed. Ubuntu/hardware validation remains for
  the operator, including Linux detached-child cleanup and ROS environment inheritance.
