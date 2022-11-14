import queue
from multiprocessing import Queue
from collections import deque
from typing import Any, List, Union, Type, Tuple


class SafeTriQueue:
    """
    A class which contains 3 queues/deques (these will both be referred to as queues). It holds an input, output, and
    action queue internally. This implements a common interface to interact with those queues regardless of the data
    structure being used internally. This was made for transferring data between tasks inside of this library. However,
    this could be utilized outside since it has a fully exposed public API and near full parameter exposure.
    """
    _supported_modes = ['process', 'thread']

    def __init__(self, input_queue: Union[deque, Queue, None] = None, output_queue: Union[deque, Queue, None] = None,
                 action_queue: Union[deque, Queue, None] = None, timeout=None, mode='Process'):
        """
        :param input_queue: A premade input queue (optional)
        :param output_queue: A premade output queue (optional)
        :param action_queue: A premade action queue (optional)
        :param timeout: Timeout of gets/puts if no timeout is provided in other methods (optional)
        :param mode: 'Process' or 'Thread' string to denote the data structure used internally (optional)
        """
        # processes which mode (process or thread) the queue will use
        self._mode = mode.lower()
        assert self._mode in SafeTriQueue._supported_modes, \
            f"Unsupported mode, supported modes: {SafeTriQueue._supported_modes}"

        # figures out which data structure to use internally
        self._queue_type = Queue
        if self._mode == 'thread':
            self._queue_type = deque

        # sets up the queues based on the queue type
        assert isinstance(input_queue, type(self._queue_type())) or input_queue is None, \
            f"Invalid type for given inputQueue, found: {type(input_queue)}, expected: {self._queue_type}"
        assert isinstance(output_queue, type(self._queue_type())) or output_queue is None, \
            f"Invalid type for given outputQueue, found: {type(output_queue)}, expected: {self._queue_type}"
        assert isinstance(action_queue, type(self._queue_type())) or action_queue is None, \
            f"Invalid type for given actionQueue, found: {type(action_queue)}, expected: {self._queue_type}"
        self._input_queue = self._queue_type() if input_queue is None else input_queue
        self._output_queue = self._queue_type() if output_queue is None else output_queue
        self._action_queue = self._queue_type() if action_queue is None else action_queue

        # adds timeout
        self._timeout = timeout

    def __eq__(self, other: 'SafeTriQueue') -> bool:
        if isinstance(other, SafeTriQueue):
            if self._input_queue == other._input_queue and self._output_queue == other._output_queue and\
                    self._action_queue == other._action_queue:
                return True
        return False

    # property interaction with timeout
    @property
    def timeout(self) -> float:
        """
        Gets the global timeout value for this TriQueue
        :return: The default timeout value
        """
        return self._timeout

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        """
        Specifies the new default timeout for get/put operations
        :param: New default timeout value
        :return: None
        """
        self._timeout = timeout

    # property for identifying the mode of the SafeQueue
    @property
    def mode(self) -> str:
        """
        The string representing the type of concurrency this TriQueue is built for (which deque/Queue is uses)
        :return: The inputted mode in the __init__ method
        """
        return self._mode

    # property for getting the type of the underlying data structure being used
    @property
    def queueType(self) -> Union[Type[Queue], Type[deque]]:
        """
        Gets the data structure type that the TriQueue uses
        :return: Type of the data structure being used internally
        """
        return self._queue_type

    # method for getting a single element from a given queue/deque
    @staticmethod
    def _get_from_queue(target_queue: Union[Queue, deque], timeout: Union[float, None] = None) -> Any:
        """
        Gets a singular element out of a given queue/deque
        :param target_queue: Queue/deque to get an element from
        :param timeout: Timeout for queue operations
        :return: The retrieved element
        """
        if isinstance(target_queue, type(Queue())):
            return target_queue.get(timeout=timeout)
        else:
            return target_queue.popleft()

    # method for adding an element to a queue with the correct method
    @staticmethod
    def _add_item_to_queue(target_queue: Union[Queue, deque], item: Any, timeout: Union[float, None] = None) -> None:
        """
        Adds a single element to a given queue/deque
        :param target_queue: Queue/deque to add an element to
        :param item: The element to add
        :param timeout: The timeout value for queue operations
        :return: None
        """
        if isinstance(target_queue, type(Queue())):
            target_queue.put(item, timeout=timeout)
        else:
            target_queue.append(item)

    # method for acquiring all elements from a multiprocessing.Queue
    @staticmethod
    def _get_all_queue(target_queue: Queue, timeout: Union[float, None] = None) -> List[Any]:
        """
        Gets all available elements from a given queue
        :param target_queue: The queue to retrieve all elements from
        :param timeout: The timeout value for queue operations
        :return: A list of all elements retrieved
        """
        item_list = []
        targetEmpty = target_queue.empty
        while not targetEmpty():
            try:
                item_list.append(target_queue.get(timeout=timeout))
            except queue.Empty:
                break
        return item_list

    # method for acquiring all elements from a collections.deque
    @staticmethod
    def _get_all_deque(target_deque: deque) -> List[Any]:
        """
        Gets all available elements from a given deque
        :param target_deque: The deque to retrieve all elements from
        :return: A list of all elements retrieved
        """
        item_list = []
        while True:
            try:
                item_list.append(target_deque.popleft())
            except IndexError:
                break
        return item_list

    # gets all items from a given queue
    def _get_all_from_queue(self, target_queue: Union[Queue, deque], timeout: Union[float, None] = None) -> List[Any]:
        """
        A wrapper function for calling _getAllDeque or _getAllQueue according to which data structure the TriQueue uses
        :param target_queue: A given Queue/deque to retrieve all elements from
        :param timeout: The timeout value for queue operations (does nothing when a deque is given)
        :return: A list of all elements retrieved
        """
        if self._queue_type == deque:
            return SafeTriQueue._get_all_deque(target_queue)
        else:
            return SafeTriQueue._get_all_queue(target_queue, self._timeout if timeout is None else timeout)

    # adds items to either queue
    def _add_to_queue(self, target_queue: Union[Queue, deque], items: Union[List[Any], Any], iterate=True) -> None:
        """
        Either adds the items parameter to the given Queue/deque or iterates over the items parameter IF it is a list
        AND iterate is set to True. Thus, Lists can be added to Queue/deques if iterate is set to False
        :param target_queue: A given Queue/deque to add elements to
        :param items: A List or single element to add
        :param iterate: Flag for iterating over lists and adding the elements present
        :return: None
        """
        if isinstance(items, List) and iterate:
            try:
                for item in items:
                    SafeTriQueue._add_item_to_queue(target_queue, item, self._timeout)
            except TypeError:
                SafeTriQueue._add_item_to_queue(target_queue, items, self._timeout)
        else:
            SafeTriQueue._add_item_to_queue(target_queue, items, self._timeout)

    # helper method which simplifies code structure for get(Input,Output,Actions)
    def _get_helper(self, target: Union[Queue, deque], get_all: bool, timeout: Union[float, None]) -> Union[List[Any],
                                                                                                            Any]:
        """
        A method for acquiring element(s) from a given Queue/deque. Calls _getAllFromQueue or _getFromQueue internally
        :param target: The Queue/deque to acquire element(s) from
        :param get_all: A flag for getting either the first or all elements from the queue
        :param timeout: A timeout value for Queue operations
        :return: Either a List of elements or a single element based on the get_all flag
        """
        if get_all:
            return self._get_all_from_queue(target, timeout=timeout)
        else:
            return SafeTriQueue._get_from_queue(target, timeout=timeout if timeout is not None else self._timeout)

    # acquires all or the first item in the input queue
    def get_input(self, get_all=False, timeout: Union[float, None] = None) -> Union[List[Any], Any]:
        """
        Gets a(n) element(s) from the input queue
        :param get_all: Flag for getting all elements
        :param timeout: Timeout value, only used if Queues are used internally ('Process' mode)
        :return: Either a List of elements or a single element
        """
        return self._get_helper(target=self._input_queue, get_all=get_all, timeout=timeout)

    # acquires all or the first item in the output queue
    def get_output(self, get_all=False, timeout: Union[float, None] = None) -> Union[List[Any], Any]:
        """
        Gets a(n) element(s) from the output queue
        :param get_all: Flag for getting all elements
        :param timeout: Timeout value, only used if Queues are used internally ('Process' mode)
        :return: Either a List of elements or a single element
        """
        return self._get_helper(target=self._output_queue, get_all=get_all, timeout=timeout)

    # acquires all or the first action in the action queue
    def get_actions(self, get_all=True, timeout: Union[float, None] = None) -> Union[List[Tuple[str, Any]], Tuple[str,
                                                                                                                  Any]]:
        """
        Gets a(n) element(s) from the action queue
        :param get_all: Flag for getting all elements
        :param timeout: Timeout value, only used if Queues are used internally ('Process' mode)
        :return: Either a List of elements or a single element
        """
        return self._get_helper(target=self._action_queue, get_all=get_all, timeout=timeout)

    # puts items into the input queue
    def add_input(self, inputs: Union[List[Any], Any], iterate=True) -> None:
        """
        Adds a(n) element(s) to the input queue
        :param inputs: A List of elements or a single element to be added
        :param iterate: Flag for whether to iterate over lists and add the items separately.
        :return: None
        """
        self._add_to_queue(self._input_queue, inputs, iterate=iterate)

    # puts items into output queue
    def add_output(self, outputs: Union[List[Any], Any], iterate=True) -> None:
        """
        Adds a(n) element(s) to the output queue
        :param outputs: A List of elements or a single element to be added
        :param iterate: Flag for whether to iterate over lists and add the items separately.
        :return: None
        """
        self._add_to_queue(self._output_queue, outputs, iterate=iterate)

    # puts an item into the action queue
    def add_action(self, actions: Union[List[Tuple[str, Any]], Tuple[str, Any]], iterate=True) -> None:
        """
        Adds a(n) element(s) to the action queue
        :param actions: A List of elements or a single element to be added
        :param iterate: Flag for whether to iterate over lists and add the items separately.
        :return: None
        """
        self._add_to_queue(self._action_queue, actions, iterate=iterate)

    # checks if a queue is empty whether it is a deque or Queue
    def _check_queue_empty(self, target_queue: Union[Queue, deque]) -> bool:
        """
        Checks if a given Queue/deque is empty or not. Uses .empty() for Queues and len() for deque
        :param target_queue: Queue/deque to check if empty
        :return: True/False if the Queue/deque is empty
        """
        if self._queue_type == deque:
            return False if len(target_queue) else True
        else:
            return target_queue.empty()

    # returns if the input queue is empty
    def inputEmpty(self) -> bool:
        """
        Checks if the input Queue/deque is empty
        :return: True if empty, else False
        """
        return self._check_queue_empty(self._input_queue)

    # returns if the output queue is empty
    def outputEmpty(self) -> bool:
        """
        Checks if the output Queue/deque is empty
        :return: True if empty, else False
        """
        return self._check_queue_empty(self._output_queue)

    # returns if the action queue is empty
    def actionEmpty(self) -> bool:
        """
        Checks if the action Queue/deque is empty
        :return: True if empty, else False
        """
        return self._check_queue_empty(self._action_queue)

    # returns if both the input and output queues are empty
    def empty(self, condition='and') -> bool:
        """
        Checks the input and output queues for being empty. Uses the inputEmpty() and outputEmpty() methods.
        If the condition is 'and', then both input and output must be empty to return True.
        If the condition is 'or', then one or more must be empty to return True.
        :param condition: Either 'and'/'or'
        :return: True if the condition is True, else False
        """
        if condition == 'and':
            return self.inputEmpty() and self.outputEmpty()
        elif condition == 'or':
            return self.inputEmpty() or self.outputEmpty()
        else:
            raise Exception(f"Unrecognized condition type in SafeQueue.empty(): {condition}")

    # method for getting the underlying object of the SafeQueue
    def getQueues(self) -> Tuple[Union[Queue, deque], Union[Queue, deque], Union[Queue, deque]]:
        """
        Gets the raw Queue/deque objects that are used internally.
        :return: A tuple of three Queue/deque (input, output, action)
        """
        return self._input_queue, self._output_queue, self._action_queue

    # mirrors multiprocessing.Queue.close method
    def close(self) -> None:
        """
        Removes all data and deallocates internal Queue/deque structures. Uses Queue.close() method for Queues and
        deque.clear() method for decks. For threaded applications is equivalent to .clear() method.
        :return: None
        """
        if self._queue_type == deque:
            self._input_queue.clear()
            self._output_queue.clear()
            self._action_queue.clear()
        else:
            self._input_queue.close()
            self._output_queue.close()
            self._action_queue.close()

    # method to clear all data from a multiprocessing queue
    @staticmethod
    def _clearQueue(q: Queue) -> None:
        # https://stackoverflow.com/questions/16461459/how-to-clear-a-multiprocessing-queue/36018632#36018632
        """
        Removes all elements from a multiprocessing Queue. Possible for race conditions to mean Queue is not fully
        clear, but this is as close as possible
        :return: None
        """
        try:
            while True:
                _ = q.get_nowait()
        except queue.Empty:
            pass

    # method to clear all queues in the TriQueue
    def clear(self) -> None:
        """
        Removes all elements from all queues present in the TriQueue. For threaded application, uses deque.clear() and
        is equivalent to the .close() method.
        :return: None
        """
        if self._queue_type == deque:
            self._input_queue.clear()
            self._output_queue.clear()
            self._action_queue.clear()
        else:
            SafeTriQueue._clearQueue(self._input_queue)
            SafeTriQueue._clearQueue(self._output_queue)
            SafeTriQueue._clearQueue(self._action_queue)
