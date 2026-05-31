#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import sys


UNSUPPORTED_TOPIC_RE = re.compile(
    r"failed to create .* bridge for topic "
    r"'/(tf|tf_static|attached_collision_object)' .*: No template specialization"
)
CHECK_PAIRS_HINT = "check the list of supported pairs with the `--print-pairs` option"


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: filter_ros1_bridge_logs.py <command> [args...]", file=sys.stderr)
        return 2

    process = subprocess.Popen(
        sys.argv[1:],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def forward_signal(signum, _frame):
        if process.poll() is None:
            process.send_signal(signum)

    signal.signal(signal.SIGTERM, forward_signal)
    signal.signal(signal.SIGINT, forward_signal)

    drop_next_check_hint = False
    assert process.stdout is not None
    for line in process.stdout:
        if UNSUPPORTED_TOPIC_RE.search(line):
            drop_next_check_hint = True
            continue
        if drop_next_check_hint and CHECK_PAIRS_HINT in line:
            drop_next_check_hint = False
            continue

        drop_next_check_hint = False
        print(line, end="", flush=True)

    return process.wait()


if __name__ == "__main__":
    os._exit(main())
