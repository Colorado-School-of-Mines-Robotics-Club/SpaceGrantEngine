from threading import Thread
from typing import Dict, Callable
from collections import defaultdict
import time
import keyboard


class KeyHandler:
    def __init__(self):
        # dict which returns a lambda function of pass if the key is not in the dict
        self._key_handlers: Dict = defaultdict(lambda: lambda: None)

        self._stopped = False

        # setup the handler
        keyboard.on_release(self.key_event)

    @property
    def stopped(self):
        return self._stopped
    
    @stopped.setter
    def stopped(self, value: bool):
        self._stopped = value

    def key_event(self, key: keyboard.KeyboardEvent):
        key = str(key)
        self._key_handlers[key]()

    def update_handler(self, key: keyboard.KeyboardEvent, handler: Callable):
        key = str(key)
        self._key_handlers[key] = handler

HANDLER = KeyHandler()

def loop():
    while not HANDLER.stopped:
        time.sleep(0.1)

if __name__ == "__main__":
    # create thread for keeping everything alive
    thread = Thread(target=loop)

    # start thread
    thread.start()

    # add the handlers for each key
    HANDLER.update_handler("a", lambda: print("a"))

    # join the thread
    thread.join()
