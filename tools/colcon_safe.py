#!/usr/bin/python3
"""Run colcon with a child watcher that works in restricted environments."""

import asyncio
import sys

from colcon_core.command import main


def configure_child_watcher() -> str:
    """Avoid ThreadedChildWatcher hangs when child-exit signals are restricted."""
    pidfd_watcher = getattr(asyncio, "PidfdChildWatcher", None)
    if pidfd_watcher is not None:
        try:
            asyncio.set_child_watcher(pidfd_watcher())
            return "PidfdChildWatcher"
        except (OSError, RuntimeError):
            pass

    asyncio.set_child_watcher(asyncio.SafeChildWatcher())
    return "SafeChildWatcher"


if __name__ == "__main__":
    configure_child_watcher()
    sys.exit(main(command_name="colcon", argv=sys.argv[1:]))
