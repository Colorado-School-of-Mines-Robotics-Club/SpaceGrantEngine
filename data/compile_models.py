import shutil
from pathlib import Path

from oakutils.blobs import compile_model

from PathfindingModel import SimpleNav

modelpath = compile_model(
    SimpleNav,
    {},
    cache=True,
    shaves=6,
)

shutil.copy(modelpath, Path("data") / "simplePathfinding.blob")
