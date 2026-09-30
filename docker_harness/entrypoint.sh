#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /harness_ws/install/setup.bash

RESET=/harness_ws/harness_tools/reset_home.py
SNAPSHOT=/tmp/scene_before.json

# Only `python3 <file>.py` runs are STLC test runs. Anything else (bash,
# python3 -c ..., reset_home.py itself) runs as-is with no reset around it.
if [[ ! ( "$1" =~ ^python3?$ && "$2" == *.py && -f "$2" && "$2" != "$RESET" ) ]]; then
    exec "$@"
fi

# STLC Manager always sends `python3 <file>.py`, but most of what it generates
# are pytest files without a `pytest.main()` call. Run with plain python3 they
# only define the test functions and exit 0 -- a pass with zero tests run.
# So if the file defines tests, run it with pytest instead. Plain scripts keep
# running exactly as before. HARNESS_RUN_MODE=script forces the old behaviour.
if [[ "${HARNESS_RUN_MODE:-auto}" != "script" ]] \
    && grep -qE '^(async def test_|def test_|class Test)' "$2"; then
    script="$2"
    shift 2
    echo "[harness] $script contains tests -> running with pytest" >&2
    cmd=(python3 -m pytest "$script" -p no:cacheprovider -rA
         --timeout="${HARNESS_TEST_TIMEOUT:-60}" "$@")
else
    cmd=("$@")
fi

# HARNESS_RESET=0 turns the post-test homing off (e.g. for debugging a test).
# The run time limit below still applies.
reset=1
[[ "${HARNESS_RESET:-1}" != "1" ]] && reset=0

set +e

# Record which planning-scene objects exist before the test, so the reset can
# remove only what the test added. A failure here must not block the test.
if (( reset )); then
    timeout 20 python3 "$RESET" snapshot --out "$SNAPSHOT" \
        || echo "[harness] snapshot failed; reset will not remove scene objects" >&2
fi

# The test is a child, not exec'd, so the reset below always runs after it.
# Bash as PID 1 ignores signals it has no trap for, so forward them: first one
# as SIGINT (Python -> KeyboardInterrupt, the script's own cleanup runs), then
# SIGTERM, then SIGKILL if the test keeps ignoring us.
signals=0
forward() {
    signals=$((signals + 1))
    case $signals in
        1) kill -INT "$child" 2>/dev/null ;;
        2) kill -TERM "$child" 2>/dev/null ;;
        *) kill -KILL "$child" 2>/dev/null ;;
    esac
}
trap forward INT TERM HUP

# Non-interactive bash starts background jobs with SIGINT ignored, and Python
# then never raises KeyboardInterrupt. Restore the default before exec'ing the test.
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    "${cmd[@]}" &
child=$!

# Overall limit on the STLC code's own run time (snapshot and reset are outside
# it). pytest-timeout only covers test bodies, not an endless loop hit while the
# file is being imported, and plain scripts have no limit at all. On expiry:
# SIGINT, then SIGTERM 30 s later, then SIGKILL 10 s after that. The timer is
# this polling loop itself, so an early exit leaves no sleep/kill process behind.
# Signals from `docker stop` are handled by forward() independently.
limit="${HARNESS_RUN_TIMEOUT:-600}"
start=$SECONDS
stage=0
while kill -0 "$child" 2>/dev/null; do
    elapsed=$((SECONDS - start))
    if (( stage == 0 && elapsed >= limit )); then
        stage=1
        echo "[harness] test timed out after ${limit}s; sending SIGINT" >&2
        kill -INT "$child" 2>/dev/null
    elif (( stage == 1 && elapsed >= limit + 30 )); then
        stage=2
        echo "[harness] test ignored SIGINT; sending SIGTERM" >&2
        kill -TERM "$child" 2>/dev/null
    elif (( stage == 2 && elapsed >= limit + 40 )); then
        stage=3
        echo "[harness] test ignored SIGTERM; sending SIGKILL" >&2
        kill -KILL "$child" 2>/dev/null
    fi
    sleep 0.5
done
wait "$child"
rc=$?
trap - INT TERM HUP

if (( stage > 0 )); then
    rc=124
fi

if (( ! reset )); then
    exit "$rc"
fi

echo "[harness] test finished with exit code $rc; resetting robot to home" >&2
timeout "${HARNESS_RESET_TIMEOUT:-240}" python3 "$RESET" reset --snapshot "$SNAPSHOT"
reset_rc=$?
if [[ $reset_rc -eq 124 ]]; then
    echo "[harness] reset: status=timeout"
fi

# The container's exit code is the TEST's result; the reset result is reported
# only in the "[harness] reset: ..." line so it never masks a test outcome.
exit "$rc"
