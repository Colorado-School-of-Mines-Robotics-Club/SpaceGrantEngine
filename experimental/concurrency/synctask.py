import time
import atexit
import string
import signal
import random
from multiprocessing import Process, Queue, Lock
from collections import deque
from collections.abc import Callable
from threading import Thread
from typing import Any, List, Tuple, Union


from safetriqueue import SafeTriQueue


# Class which defines all required methods for job classes. Used for process, thread, and coroutine jobs
# Can also be used to define custom job types and still have them managed by the JobManager
class SyncTask:
    """
    Implements a task which processes a single function communicating over a
    thread or process queue system. The queue system is implemented using the TriQueue class. This class 
    """
    _supported_types = [Process, Thread]
    _job_types = ['process', 'thread']

    # simple constructor for managing properties
    def __init__(self, target: Callable, target_args: Tuple, lock: Union[Lock, None] = None,
                 static_obj_builder: Union[Callable, None] = None, static_obj_builder_args: Union[Tuple, None] = None,
                 name: Union[str, None] = None, daemon: bool = False, timeout: Union[float, None] = None,
                 job_type='Process'):
        # checking job type
        self._job_type = job_type.lower()
        assert self._job_type in SyncTask._job_types

        # property attributes
        random_id = ''.join(random.choice(string.digits) for i in range(10))
        self._name = name + ' ' + random_id if name is not None else random_id
        self._timeout = timeout
        self._daemon = daemon

        # attributes for flow of job
        self._stopped = False
        self._running = False
        self._paused = False
        self._sleeping = False

        # target attributes which Job executes on
        self._target = target
        self._target_args = target_args

        # lock for halting execution
        self._lock = lock if lock is not None else Lock()

        # sets up static (not-pickleable) object builder
        self._static_obj_builder = static_obj_builder
        self._static_obj_builder_args = static_obj_builder_args

        # makes the safe queue
        self._queue = SafeTriQueue()

        # form the process/thread to be executed
        assert self._job_type in SyncTask._job_types
        if self._job_type == 'process':
            self._executor = Process(name=name, target=self._run, args=(), daemon=self._daemon)
        elif self._job_type == 'thread':
            self._executor = Thread(name=name, target=self._run, args=(), daemon=self._daemon)

        # registers close method
        atexit.register(self._join)

    # properties of the job
    @property
    def name(self) -> Union[str, None]:
        return self._name

    @property
    def timeout(self) -> Union[float, None]:
        return self._timeout

    @timeout.setter
    def timeout(self, val: float) -> None:
        self._timeout = val

    @property
    def daemon(self) -> bool:
        return self._daemon

    @property
    def alive(self) -> bool:
        return self._executor.is_alive()

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def running(self) -> bool:
        return self._running

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def sleeping(self) -> bool:
        return self._sleeping

    # methods for interacting with the target function
    # method for executing the job, this method contains an execution loop for a target func
    def _run(self, inputQ: Union[Queue, deque], outputQ: Union[Queue, deque], actionQ: Union[Queue, deque], lock: Lock):
        # reassign the lock
        self._lock = lock
        # rebuild the SafeQueue using the individual queues. Queues must be passed as arguments on Unix
        self._queue = SafeTriQueue(input_queue=inputQ, output_queue=outputQ, action_queue=actionQ,
                                   timeout=self._timeout)
        # form the static (non-pickleable) objects
        if self._static_obj_builder is not None:
            static_objects = self._static_obj_builder(*self._static_obj_builder_args)
            target_args = (self._queue,) + tuple(static_objects) + self._target_args
        else:
            target_args = (self._queue,) + self._target_args
        # defines signal catchers
        self._handle_signals()
        # execution loop
        while not self._stopped:
            self._parse_action_queue()
            # handle a stopped flag from the action queue
            if self._stopped:
                break
            # process target
            outputs = self._target(*target_args)
            self._add_output(outputs)
        self._join()

    def start(self) -> None:  # starts execution of the job
        """
        Starts the execution of the task
        :return: None
        """
        self._update_runtime_props(running=True)
        self._executor.start()

    def stop(self) -> None:  # stops execution of the job, does not close thread/process
        """
        Stops the task
        :return: None
        """
        self._update_runtime_props(running=False, stopped=True)
        self._queue.add_action(("stop", None))

    def pause(self) -> None:  # pauses execution of the job
        """
        Pauses a task
        :return: None
        """
        if not self._paused:
            self._lock.acquire()
            self._update_runtime_props(running=False, paused=True)

    def resume(self) -> None:  # resumes execution of the job
        """
        Resumes a stopped task
        :return: None
        """
        self._lock.release()
        self._update_runtime_props(running=True, paused=False, stopped=False)

    def _join(self, timeout: Union[float, None] = None):
        self._update_runtime_props(running=False, paused=False, stopped=True)
        try:  # catches the exception if the task was never started
            self._executor.join(timeout=timeout)
        except AssertionError:
            pass
        if self._job_type == 'process':
            try:  # close the executor if it is a process, once a process has been closed it could raise a ValueError
                self._executor.close()
            except ValueError:
                pass

    def join(self, timeout: Union[float, None]) -> None:  # joins the job process/thread
        """
        Joins the executor of the task
        :param timeout: How long to wait for the join to occur
        :return: None
        """
        self._update_runtime_props(running=False, paused=False, stopped=False)
        self._executor.join(timeout=timeout)

    def _sleep(self, sleep_time: float) -> None:
        self._update_runtime_props(running=False, sleeping=True)
        time.sleep(sleep_time)
        self._update_runtime_props(running=True, sleeping=False)

    def sleep(self, sleep_time: float) -> None:  # sleeps the job for the given time in seconds
        """
        Sleeps the task
        :param sleep_time: The time in seconds for the task to sleep
        :return: None
        """
        self._queue.add_action(('sleep', sleep_time))

    def _update_runtime_props(self, stopped: Union[bool, None] = None, running: Union[bool, None] = None,
                              paused: Union[bool, None] = None, sleeping: Union[bool, None] = None):
        if not self.alive:
            stopped, running, paused, sleeping = False, False, False, False
        self._stopped = stopped if stopped is not None else self._stopped
        self._running = running if running is not None else self._running
        self._paused = paused if paused is not None else self._paused
        self._sleeping = sleeping if sleeping is not None else self._sleeping

    # signal handling functions
    def _handle_signals(self) -> None:  # required internal method for handling signals
        signal.signal(signal.SIGINT, self._handle_sigint)

    def _handle_sigint(self, sig, frame):
        self.stop()
        try:  # use the internal join and stop methods, process methods could fail with valueError if already closed
            self._join()
        except ValueError:
            pass

    # handles a singular action
    def _parse_action(self, action: Tuple[str, Any]) -> None:
        action_str = action[0].lower()
        if action_str == 'stop':
            self._stopped = True
        elif action_str == 'sleep':
            self._sleep(action[1])

    # required internal method for handling an internal action queue
    def _parse_action_queue(self) -> None:
        actions = self._queue.get_actions(self._timeout)
        if isinstance(actions, List):
            for action in actions:
                self._parse_action(action)
        else:
            self._parse_action(actions)

    # methods for interacting with the data input/output
    def add_input(self, inputs: Union[List[Any], Any], iterate: bool = False) -> None:
        """
        Adds inputs to the given task.
        :param inputs: A list of inputs or a single input
        :param iterate: If true put all inputs in a list into the queue independently. If false put the list in the
        queue as a single input
        :return: None
        """
        self._queue.add_input(inputs=inputs, iterate=iterate)

    def _add_output(self, outputs: Union[List[Any], Any], iterate: bool = False) -> None:
        self._queue.add_output(outputs=outputs, iterate=iterate)

    def get_output(self, get_all=False, timeout: Union[float, None] = None) -> Union[List[Any], Any]:
        """
        Gets the output from the task
        :param get_all: Flag for acquiring all outputs or just the first
        :param timeout: The time to wait, blocks if None
        :return: A list of outputs, or the first output
        """
        return self._queue.get_output(get_all=get_all, timeout=timeout)
