import json
from typing import Dict


class JSONReader:
    """
    Reads in a given JSON file and creates a dictionary structure
    """

    def __init__(self, config_file: str):
        assert config_file.endswith(".json")
        self._filename: str = config_file
        temp_data: Dict = {}
        with open(config_file, "rb") as file:
            temp_data = json.load(file)
        temp_data = JSONReader._flatten_json_dict(temp_data)

        self._data: Dict = {}
        for key in temp_data:
            self._data[
                self._filename[0 : len(self._filename) - 5] + "_" + key
            ] = temp_data[key]

    @staticmethod
    def _flatten_json_dict(input_dict: Dict):
        flattened_dict = {}
        for key in input_dict:
            val = input_dict[key]
            if isinstance(val, dict):
                sub_dict = JSONReader._flatten_json_dict(val)
                for sub_key in sub_dict:
                    flattened_dict[str(key) + "_" + str(sub_key)] = sub_dict[sub_key]
            else:
                flattened_dict[str(key)] = val
        return flattened_dict

    @property
    def data(self) -> Dict:
        """
        Get the dictionary defined by the .json configuration file
        """
        return self._data
        