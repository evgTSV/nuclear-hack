from os import getenv
from pipe import Pipe
from dotenv import load_dotenv

load_dotenv()

offsets = (
        Pipe(getenv("ROBOT_POS_OFFSETS"))
        | (lambda s: s.strip())
        | (lambda s: s.split())
        | (lambda p: map(float, p))
        | list
).get()

corners_markers = (
        Pipe(getenv("CORNERS_MARKERS"))
        | (lambda s: s.strip())
        | (lambda s: s.split())
        | (lambda p: map(int, p))
        | list
).get()

construct_markers = (
        Pipe(getenv("CONSTRUCT_MARKERS"))
        | (lambda s: s.strip())
        | (lambda s: s.split())
        | (lambda p: map(int, p))
        | list
).get()

cam_id = int(getenv("CAM_ID"))

marker_z = float(getenv("MARKER_Z"))

field_z = float(getenv("FIELD_Z"))