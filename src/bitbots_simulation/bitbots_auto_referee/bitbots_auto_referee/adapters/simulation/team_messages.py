"""Receive the additional loopback copy of each logical team message."""

import socket

LOCAL_TEAM_PORT_BASE = 4000
LOCAL_TEAM_PORT_STRIDE = 10
LOCAL_COUNT_PORT_OFFSET = 9
MAX_BATCH = 256


def team_message_port(team_id: int) -> int:
    """Match the counting destination in the simulation's team_comm launch config."""
    return LOCAL_TEAM_PORT_BASE + LOCAL_TEAM_PORT_STRIDE * team_id + LOCAL_COUNT_PORT_OFFSET


class TeamMessageReceiver:
    def __init__(self, team_ids: tuple[int, ...]):
        self._sockets: dict[int, socket.socket] = {}
        try:
            for team in team_ids:
                receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sockets[team] = receiver
                receiver.setblocking(False)
                receiver.bind(("127.0.0.1", team_message_port(team)))
        except OSError:
            self.close()
            raise

    def receive_counts(self) -> dict[int, int]:
        """Drain a bounded batch per team without decoding the payload."""
        counts = {}
        for team, receiver in self._sockets.items():
            count = 0
            for _ in range(MAX_BATCH):
                try:
                    receiver.recvfrom(65535)
                except BlockingIOError:
                    break
                count += 1
            if count:
                counts[team] = count
        return counts

    def close(self) -> None:
        for receiver in self._sockets.values():
            receiver.close()
        self._sockets.clear()
