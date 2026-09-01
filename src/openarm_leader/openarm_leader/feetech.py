"""Feetech STS3215 리더암 버스 읽기. scservo_sdk 는 이 모듈 안에서만 쓴다."""


class FeetechBus:
    """리더 서보의 현재 위치를 tick 으로 읽어오는 시리얼 버스."""

    def __init__(self, port, baudrate, protocol_end, present_position_address):
        # sliders 모드에서는 SDK 없이 돌아야 하므로 import 를 여기서 한다.
        from scservo_sdk import (
            COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler)

        self._comm_success = COMM_SUCCESS
        self._group_read_factory = GroupSyncRead
        self._address = present_position_address
        self._length = 2

        try:
            self._port = PortHandler(port)
            opened = self._port.openPort()
        except OSError as error:
            raise RuntimeError(f'{port}: 포트를 열지 못했다 ({error})') from error
        if not opened:
            raise RuntimeError(f'{port}: 포트를 열지 못했다.')
        if not self._port.setBaudRate(baudrate):
            self._port.closePort()
            raise RuntimeError(f'{port}: baudrate {baudrate} 설정에 실패했다.')
        self._packet = PacketHandler(protocol_end)
        self._group = None
        self._group_ids = None

    def _group_for(self, ids):
        wanted = [int(servo_id) for servo_id in ids]
        if self._group_ids != wanted:
            group = self._group_read_factory(
                self._port, self._packet, self._address, self._length)
            for servo_id in wanted:
                group.addParam(servo_id)
            self._group = group
            self._group_ids = wanted
        return self._group

    def read_positions(self, ids):
        """ids 순서대로 현재 위치 tick 을 읽는다. 한 개라도 실패하면 None."""
        group = self._group_for(ids)
        if group.txRxPacket() != self._comm_success:
            return None
        ticks = []
        for servo_id in self._group_ids:
            if not group.isAvailable(servo_id, self._address, self._length):
                return None
            ticks.append(group.getData(servo_id, self._address, self._length))
        return ticks

    def read_position(self, servo_id):
        """서보 하나의 현재 위치 tick. 실패하면 None."""
        value, result, _error = self._packet.read2ByteTxRx(
            self._port, int(servo_id), self._address)
        if result != self._comm_success:
            return None
        return value

    def close(self):
        if self._port is not None:
            self._port.closePort()
            self._port = None
