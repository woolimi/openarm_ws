"""Feetech STS3215 리더암 버스 접근. scservo_sdk 는 이 모듈 안에서만 쓴다."""

#: STS 시리즈 제어 테이블 주소. 프로토콜 상수라 config 가 아니라 여기 둔다.
ID_ADDRESS = 5
LOCK_ADDRESS = 55


class FeetechBus:
    """리더 서보의 현재 위치를 tick 으로 읽어오는 시리얼 버스."""

    def __init__(self, port, baudrate, protocol_end, present_position_address):
        # sliders 모드에서는 SDK 없이 돌아야 하므로 import 를 여기서 한다.
        try:
            from scservo_sdk import (
                COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler)
        except ImportError as error:
            raise RuntimeError(
                'scservo_sdk 를 찾지 못했다. venv 를 켠 채로 colcon 을 다시 빌드하라.'
            ) from error

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

    def ping(self, servo_id):
        """서보 응답 확인. 성공하면 모델 번호, 실패하면 None."""
        model, result, _error = self._packet.ping(self._port, int(servo_id))
        if result != self._comm_success:
            return None
        return model

    def scan(self, id_range):
        """id_range 를 차례로 ping 해 {id: 모델 번호} 로 돌려준다."""
        found = {}
        for servo_id in id_range:
            model = self.ping(servo_id)
            if model is not None:
                found[int(servo_id)] = model
        return found

    def write_u8(self, servo_id, address, value):
        """1바이트 레지스터 쓰기. 성공 여부를 돌려준다."""
        result, error = self._packet.write1ByteTxRx(
            self._port, int(servo_id), int(address), int(value))
        return result == self._comm_success and error == 0

    def set_servo_id(self, current_id, new_id):
        """서보 id 를 바꾼다. EEPROM 잠금을 풀고 쓰고 다시 잠근다."""
        if not self.write_u8(current_id, LOCK_ADDRESS, 0):
            return False
        if not self.write_u8(current_id, ID_ADDRESS, new_id):
            self.write_u8(current_id, LOCK_ADDRESS, 1)
            return False
        self.write_u8(new_id, LOCK_ADDRESS, 1)
        return self.ping(new_id) is not None

    def close(self):
        if self._port is not None:
            self._port.closePort()
            self._port = None
