import logging


class SG_Logger:

    # constructor to setup the logging for all the nodes
    # when classes inherit this one, logs go to a file(important_logs.log) and to the stderr console
    def __init__(self) -> None:
        logging.basicConfig(
            filename="/home/pi/SpaceGrantEngine/log/important_logs.log",
            level=logging.INFO,
            format="%(levelname)s:%(filename)s:%(lineno)d (@%(asctime)s) %(message)s",
        )
        logging.getLogger().addHandler(logging.StreamHandler())
