# Experiment manager design

Approved direction: TCP 5006 manages trials; existing UDP 5005 ASCII commands
remain unchanged. An Ubuntu standard-library supervisor survives individual
`run_session.sh` processes and starts one full session at a time.

## Wire protocol

Every TCP message is ASCII terminated by LF, maximum 1024 bytes including LF.
`START,<request_id>,<participant>,<session>` uses a unique 1–64 character
alphanumeric/underscore/hyphen request ID, participant `P[0-9]+`, and session
`0|[1-9][0-9]*`. Responses: `STARTING,id,P01,0`, `READY,id,P01,0`,
`DONE,id,P01,0`, `ERROR,id,reason`, or `BUSY,id`. `STATUS,id` and repeating an
identical START replay the latest status. Reusing an ID with different metadata
returns ERROR without affecting the original request. The request ledger lasts
for this supervisor process, including TCP reconnects, not across restarts.

`FINISH_EXPERIMENT` rejects further STARTs and responds `FINISHING` during an
active trial. The trial continues normally; after its terminal status, return
`EXPERIMENT_FINISHED` and exit. When idle, finish immediately. A disconnected
client can reconnect and query status; it never automatically restarts a trial.
Only one TCP client is served at once, and the first valid START binds the
experiment to that client's IP. This is a trusted, direct laboratory link.

## Child lifecycle and readiness

Each child inherits a private `FMP_STATUS_DIR`. The UDP bridge writes
`bridge_ready` only after binding its socket; session_manager writes
`controller_ready` only after a successful controller switch and a WAITING
state observation. The supervisor then creates `accept_commands` and sends
READY. The bridge drops packets before this gate; it does not buffer commands
for a future trial. Standalone sessions without the environment variable keep
their current behavior.

session_manager writes `completed` only after DONE, CSV append, and the final
recording delay. DONE requires this marker and a zero child exit status after
process cleanup. A missing marker, launch failure, or startup timeout yields
ERROR, never DONE. Preserve roslaunch's exit code in run_session.sh. Startup
timeout interrupts child processes, waits, escalates bounded cleanup
if necessary, and reports ERROR after cleanup. Linux /proc scans match the exact
inherited FMP_STATUS_DIR environment entry, including detached ROS nodes. If any
live trial processes remain, abort rather than allow another trial. Ctrl+C also cleans up children.
The C++ controller and its motion limits remain unchanged.

## Windows tools and validation

Keep CommandSender import compatibility. Add a TCP ExperimentClient, a managed
pattern mode, and retain scripts/fmp_udp_keyboard.py as the keyboard entry point. Managed mode
prompts for IDs, waits for READY, sends UDP only during the active trial, and
returns to the next-ID prompt after DONE or ERROR. Keyboard holds 1 for
clockwise, 2 for anti-clockwise, 0 for stop, and Q/Esc requests graceful
experiment finish. Preserve focus checks and stop on key release. The batch launcher invokes the keyboard script. Explicit
`--udp-only` supports the existing standalone UDP debugging workflow.

Use standard-library unittest, real loopback TCP/UDP sockets and fake child
processes; no robot or ROS startup is authorized. Verify repeated trials,
readiness gates, failures, deduplication, reconnect, framing, and deferred finish.
Copy the three tested Windows files back to the user-specified parent directory
only with the required filesystem escalation.
