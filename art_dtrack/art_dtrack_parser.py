from dataclasses import dataclass, field
import time
import re
import numpy as np


@dataclass()
class DtrackBody:
    id: int = -1
    """ The id of the body. Starts with 0. """
    x: float = 0
    """ Translation of the body along the x axis in millimeters. """
    y: float = 0
    """ Translation of the body along the y axis in millimeters. """
    z: float = 0
    """ Translation of the body along the z axis in millimeters. """
    rx: float = 0
    """ Rotation of the body around the x axis in degrees. """
    ry: float = 0
    """ Rotation of the body around the y axis in degrees. """
    rz: float = 0
    """ Rotation of the body around the z axis in degrees. """
    rot: np.ndarray = field(default_factory=lambda: np.eye(3, 3))
    """ The bodies orientation expressed as a 3x3 Rotation matrix. """


@dataclass()
class DtrackMessage:
    frame: int = -1
    """ The frame increasing with each measurement. """
    timestamp: float = -1
    """ The raw timestamp received from the system (resets midnight). """
    timestamp_full: float = -1
    """ The full timestamp including the current date. """
    num_calibrated_bodies: int = -1
    """ Number of calibrated bodies. """
    bodies: list[DtrackBody] = field(default_factory=list)
    """ The list of actually tracked bodies. The length may be smaller than `num_bodies_total`. """


def parseDtrackLine(line: str) -> DtrackMessage:
    msg = DtrackMessage()
    # iterate over each data row
    for row in line.splitlines():
        data = row.split(" ")
        if data[0] == "fr":
            msg.frame = int(data[1])
        elif data[0] == "ts":
            msg.timestamp = float(data[1])
            time_now = time.localtime(time.time())
            time_tuple = (
                time_now.tm_year,
                time_now.tm_mon,
                time_now.tm_mday,
                0,
                0,
                0,
                time_now.tm_wday,
                time_now.tm_yday,
                0,
            )
            seconds_to_last_midnight = time.mktime(time_tuple)
            msg.timestamp_full = seconds_to_last_midnight + msg.timestamp
        elif data[0] == "6dcal":
            msg.num_calibrated_bodies = int(data[1])
        elif data[0] == "6d":
            # tells us how many blocks to expect
            num_tracked = int(data[1])
            # each block contains one data list enclosed by square brackets
            blocks = re.findall(r"(?<=\[)[^\]]*[^\[]*(?=\])", row)
            blocks = [block.split(" ") for block in blocks]
            # three blocks in a row describe one body: id, 6DOF, rot matrix
            for i in range(num_tracked):
                block_id, block_6d, block_3x3 = blocks[i * 3 : i * 3 + 3]
                body = DtrackBody()
                body.id = int(block_id[0])
                body.x, body.y, body.z, body.rx, body.ry, body.rz = [
                    float(n) for n in block_6d
                ]
                body.rot = np.array([float(n) for n in block_3x3]).reshape([3, 3])
                msg.bodies.append(body)
    return msg
