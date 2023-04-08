# # from configparser import ConfigParser
# from typing import Dict


# class INIReader:
#     """
#     Reads in a given INI file and creates a dictionary structure
#     """

#     def __init__(self, config_file: str):
#         assert config_file.endswith(".ini")
#         self._filename: str = config_file
#         self._data: Dict = {}

#     @property
#     def data(self) -> Dict:
#         """
#         Get the dictionary defined by the .ini configuration file
#         """
#         return self._data
