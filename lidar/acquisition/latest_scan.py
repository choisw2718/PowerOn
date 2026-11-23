"""Read complete rotations independently of the controller's processing time."""

from contextlib import closing
import threading
import time


OVERDUE_SCAN = object()


def latest_scans(rotations, stop, reader_stop, deadline, *, gap_grace=None):
    ready = threading.Condition()
    newest = failure = None
    finished = False

    def read():
        nonlocal newest, failure, finished
        try:
            with closing(rotations):
                for frame in rotations:
                    if reader_stop.is_set() or stop.is_set():
                        break
                    with ready:
                        newest = frame
                        ready.notify()
        except Exception as error:
            with ready:
                failure = error
        finally:
            with ready:
                finished = True
                ready.notify_all()

    worker = threading.Thread(target=read, name="lidar-rotations", daemon=True)
    worker.start()
    next_due = deadline
    try:
        while not stop.is_set() and time.monotonic() < deadline:
            with ready:
                if failure is not None:
                    raise failure
                if newest is None:
                    if finished:
                        return
                    wait = min(deadline, next_due) - time.monotonic()
                    if wait > 0 or gap_grace is None:
                        ready.wait(min(0.05, max(0.0, wait)))
                        continue
                    frame, next_due = OVERDUE_SCAN, deadline
                else:
                    frame, newest = newest, None
                    if gap_grace is not None:
                        next_due = time.monotonic() + (frame.end - frame.start) + gap_grace
            yield frame
    finally:
        reader_stop.set()
        worker.join(timeout=1.0)
