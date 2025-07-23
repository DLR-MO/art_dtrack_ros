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
    """ The body's orientation expressed as a 3x3 Rotation matrix. """


@dataclass()
class DtrackMeasurementTool:
    id: int = -1
    """ The id of the body. Starts with 0. """
    x: float = 0
    """ Translation of the body along the x axis in millimeters. """
    y: float = 0
    """ Translation of the body along the y axis in millimeters. """
    z: float = 0
    """ Translation of the body along the z axis in millimeters. """
    rot: np.ndarray = field(default_factory=lambda: np.eye(3, 3))
    """ The measurement tool's orientation expressed as a 3x3 Rotation matrix. """
    button: int = 0
    """ Status of the buttons as a decimal number,
    this is a binary encoding of the status of each button: bit 0 for button 1, bit 1 for button 2, ..."""
    covariance: np.ndarray = field(default_factory=lambda: np.eye(3, 3))
    """ The 3x3 symmetric covariance matrix of the measurement tool tip in millimeters squared. """


@dataclass()
class DtrackMeasurementToolReference:
    id: int = -1
    """ The id of the body. Starts with 0. """
    x: float = 0
    """ Translation of the body along the x axis in millimeters. """
    y: float = 0
    """ Translation of the body along the y axis in millimeters. """
    z: float = 0
    """ Translation of the body along the z axis in millimeters. """
    rot: np.ndarray = field(default_factory=lambda: np.eye(3, 3))
    """ The reference body's orientation expressed as a 3x3 Rotation matrix. """


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
    num_defined_measurement_tools: int = -1
    """ Number of defined measurement tools. """
    num_defined_measurement_tool_references: int = -1
    """ Number of defined measurement tool reference bodies. """
    bodies: list[DtrackBody] = field(default_factory=list)
    """ The list of actually tracked bodies. The length may be smaller than `num_calibrated_bodies`. """
    measurement_tools: list[DtrackMeasurementTool] = field(default_factory=list)
    """ The list of tracked measurement tools. """
    measurement_tool_reference_bodies: list[DtrackMeasurementToolReference] = field(
        default_factory=list
    )
    """ The list of tracked measurement tool reference bodies. The length may be smaller than `num_defined_measurement_tool_references`. """


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
                body.rot = np.array([float(n) for n in block_3x3]).reshape([3, 3]).T
                msg.bodies.append(body)
        elif data[0] == "6dmt2":
            # tells us how many blocks to expect
            msg.num_defined_measurement_tools = int(data[1])
            num_tracked = int(data[2])
            # each block contains one data list enclosed by square brackets
            blocks = re.findall(r"(?<=\[)[^\]]*[^\[]*(?=\])", row)
            blocks = [block.split(" ") for block in blocks]
            # five blocks in a row describe one measurement tool: id, position, rot matrix, button status, covariance matrix
            for i in range(num_tracked):
                block_id, block_position, block_3x3, block_button, block_covariance = (
                    blocks[i * 5 : i * 5 + 5]
                )
                body = DtrackMeasurementTool()
                body.id = int(block_id[0])
                body.x, body.y, body.z = [float(n) for n in block_position]
                body.rot = np.array([float(n) for n in block_3x3]).reshape([3, 3]).T
                body.button = block_button
                arr = block_covariance
                body.covariance = np.array(
                    [
                        [arr[0], arr[1], arr[2]],
                        [arr[1], arr[3], arr[4]],
                        [arr[2], arr[3], arr[5]],
                    ]
                )
                msg.measurement_tools.append(body)
        elif data[0] == "6dmtr":
            # tells us how many blocks to expect
            msg.num_defined_measurement_tool_references = int(data[1])
            num_tracked = int(data[2])
            # each block contains one data list enclosed by square brackets
            blocks = re.findall(r"(?<=\[)[^\]]*[^\[]*(?=\])", row)
            blocks = [block.split(" ") for block in blocks]
            # three blocks in a row describe one body: id, position, rot matrix
            for i in range(num_tracked):
                block_id, block_position, block_3x3 = blocks[i * 3 : i * 3 + 3]
                body = DtrackMeasurementToolReference()
                body.id = int(block_id[0])
                body.x, body.y, body.z = [float(n) for n in block_position]
                body.rot = np.array([float(n) for n in block_3x3]).reshape([3, 3]).T
                msg.measurement_tool_reference_bodies.append(body)
    return msg


# fr 46565
# ts 9800.056909
# 6dcal 3
# 6d 1
#   [0 1.000][2588.621 -280.866 886.056 17.3456 -33.6350 -51.4519]
#       [0.518842 -0.849428 0.096310 0.651151 0.465681 0.599286 -0.553900 -0.248222 0.794720]
# 6dmt2 2 2
#   [0 -1.000 1 2.000][0.000 0.000 0.000]
#       [0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000]
#       [0][0.000e+00 0.000e+00 0.000e+00 0.000e+00 0.000e+00 0.000e+00]
#   [1 1.000 1 2.000][-177.273 -650.842 110.498]
#       [-0.561205 0.810283 0.168789 0.809852 0.495484 0.314064 0.170848 0.312948 -0.934277]
#       [0][1.082e-01 -6.468e-04 4.842e-02 6.498e-02 9.944e-02 6.639e-01]
# 6dmtr 2 1
#   [1 1.000][2463.430 -106.744 874.478]
#       [0.885991 0.463701 0.000865 -0.463686 0.885942 0.010090 0.003913 -0.009340 0.999949]

# 6DOF Body
# [id qu][sx sy sz η θ φ][b0 b1 b2 b3 b4 b5 b6 b7 b8 ]
# ID number (id, starting with 0), quality value (qu, unused),
# Position (sx sy sz ), orientation angles (η θ φ),
# The nine values b0 . . . b8 form the rotation matrix R

# Measurement Tool
# [id qu nbt rd][sx sy sz ][b0 b1 b2 b3 b4 b5 b6 b7 b8 ][bt][σ11 σ12 σ13 σ22 σ23 σ33 ]
# ID number (id, starting with 0), quality value (qu, see below), number of buttons
#   (nbt) and the radius of the Measurement Tool tip sphere (rd, if applicable),
#   The quality value (qu) can only become the values 1.0 and −1.0. −1.0 means that the
#   Measurement Tool is not visible at the moment.Then dummy values are used for position (zero)
#   and orientation (zero matrix!). Still information about the button is valid.
# Measured position (sx sy sz ) of the tip,
# Orientation of the Measurement Tool tip given as rotation matrix (bi , like standard bodies),
# Button status (bt )
# Covariance matrix (σij ) of the position of the tool tip (in mm2 ).
#       σ11 σ12 σ13
# Σ = ( σ12 σ22 σ23 )
#       σ13 σ23 σ33

# Measurement Tool Reference Body
# [id qu][sx sy sz ][b0 b1 b2 b3 b4 b5 b6 b7 b8 ]
