from dataclasses import dataclass
from enum import Enum, auto
from typing import Union
from models import Point3
from singleton_logger import SingletonLogger

class RbCmdKind(Enum):
    MOVE = auto()
    VACUUM_ON = auto()
    VACUUM_OFF = auto()
    SET_ROTATE = auto()

@dataclass
class Move:
    pos: Point3
    tag: RbCmdKind = RbCmdKind.MOVE

@dataclass
class VacuumOn:
    tag: RbCmdKind = RbCmdKind.VACUUM_ON

@dataclass
class VacuumOff:
    tag: RbCmdKind = RbCmdKind.VACUUM_OFF

@dataclass
class SetRotate:
    deg: int
    tag: RbCmdKind = RbCmdKind.SET_ROTATE
    
RbCmd = Union[Move, VacuumOn, VacuumOff, SetRotate]
logger = SingletonLogger()

def command_to_str(command: RbCmd):
    match command.tag:
        case RbCmdKind.MOVE:
            pos = command.pos
            return f"MOVE_TO {pos.x} {pos.y} {pos.z}"
        case RbCmdKind.SET_ROTATE:
            return f"TOOL_ROTATE_TO {command.deg}"
        case RbCmdKind.VACUUM_ON:
            return "TOOL_VACUUM_ON"
        case RbCmdKind.VACUUM_OFF:
            return "TOOL_VACUUM_OFF"
        case _:
            logger.error("Такой команды нет!")
            return ""