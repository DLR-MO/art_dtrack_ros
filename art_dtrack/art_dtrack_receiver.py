# SPDX-FileCopyrightText: 2025 Fabian Wieczorek, German Aerospace Center (DLR)
#
# SPDX-License-Identifier: MIT

import socket
from .art_dtrack_parser import DtrackMessage, parseDtrackLine, DtrackBody


class ArtDtrackReceiver:
    """Configures the socket to receive messages."""

    def __init__(self, ip, port=4100, con_timeout=3.0):
        self.ip = ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(con_timeout)
        self.socket_initialized = False

    def receive(self, buffer_size=1024) -> DtrackMessage:
        """Receives one udp package, parses it and returns"""
        if not self.socket_initialized:
            self.socket.bind((self.ip, self.port))
            self.socket_initialized = True
        udp_data, _ = self.socket.recvfrom(buffer_size)
        return parseDtrackLine(udp_data.decode())
