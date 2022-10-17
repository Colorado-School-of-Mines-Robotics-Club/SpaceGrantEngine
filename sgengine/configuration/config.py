from typing import Dict

import imgui

from .json_reader import JSONReader
from .ini_reader import INIReader


class Config:
    """
    Class for loading configuration files and editing their values dynamically at runtime.
    Supports the use of .json and .ini input files.
    Constructor takes any number of configuration files
    """

    def __init__(self, *args):
        for config_file in args:
            assert isinstance(config_file, str)
            assert config_file.endswith(".json") or config_file.endswith(".ini")

        self._files = [file for file in args]
        self._data: Dict = {}

        for file in self._files:
            if file.endswith(".json"):
                data = JSONReader(file).data
            elif file.endswith(".ini"):
                data = INIReader(file).data
            self._data.update(data)

    @property
    def data(self) -> Dict:
        """
        Get the dictionary defined by the configuration file
        """
        return self._data

    # method to define the menu bar portion
    def menu_bar(self):
        """
        A method which is meant to be run inside of an imgui rendering loop.
        This will render a menu bar option for the config class
        """
        if imgui.begin_menu("Config", True):

            imgui.end_menu()
            