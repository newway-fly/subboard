# task_queue.py
"""
task_queue.py

Unified Task Queue Module (FIFO)
- All asynchronous work is enqueued as tasks to ensure the main loop remains non-blocking.
- Task format: (callable, args_tuple, kwargs_dict)
- Provides `run_one()` and `run_all(limit)` interfaces. The main loop periodically calls `run_one` or `run_all`.
- Exceptions during task execution are caught and logged; they do not stop the queue from running.
"""

import micropython
from log_system import log, INFO

# small helper: enable finalization of exceptions to help debugging in micropython environment
micropython.alloc_emergency_exception_buf(100)

class TaskQueue:
    def __init__(self, max_size=256):
        self.queue = []
        self.max_size = max_size

    def add_task(self, func, *args, **kwargs):
        """Put the task into the queue; if the queue is full, discard it and return False (logging or alerting can be done externally)"""
        if len(self.queue) >= self.max_size:
            # The strategy for discarding the oldest or newest task can be adjusted here; current strategy: reject new tasks
            # print("TaskQueue full, dropping task:", func.__name__)
            log(INFO, f"TaskQueue full, dropping task, {func.__name__}") 
            return False
        self.queue.append((func, args, kwargs))
        return True

    def run_one(self):
        """Execute a task (FIFO)"""
        if not self.queue:
            return False
        func, args, kwargs = self.queue.pop(0)
        try:
            func(*args, **kwargs)
        except Exception as e:
            # 记录异常但不让系统停止
            # print("Task execution error:", e)
            log(INFO, f"Task execution error:", e) 
        return True

    def run_all(self, limit=50):
        """
        Execute up to `limit` tasks (to prevent taking too long at once)
        - `limit` is adjustable; the main loop can call `run_all(limit)` each time 
          to balance responsiveness and processing capacity
        """
        ran = 0
        while ran < limit and self.queue:
            self.run_one()
            ran += 1
        return ran

    def __len__(self):
        return len(self.queue)
