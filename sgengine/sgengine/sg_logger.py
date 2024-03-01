import logging
import os


class SG_Logger:

    # constructor to setup the logging for all the nodes
    # when classes inherit this one, logs go to a file(sg_logs.log) and to the stderr console
    def __init__(self) -> None:
        logging.basicConfig(
            filename=os.getenv("SG_LOG_PATH", "log/sg_logs.log"),
            level=logging.INFO,
            format="%(levelname)s:%(filename)s:%(lineno)d (@%(asctime)s) %(message)s",
        )
        logging.getLogger().addHandler(logging.StreamHandler())
