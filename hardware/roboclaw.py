import random
import serial
import time


class Roboclaw:
    """
    Python 3 RoboClaw packet-serial interface.

    This version supports the commands required by your robot:
        Open
        Close
        DutyM1
        DutyM2
        DutyM1M2
        ForwardM1
        BackwardM1
        ForwardM2
        BackwardM2
        ReadVersion
        ReadMainBatteryVoltage
        ReadLogicBatteryVoltage
        ReadEncM1
        ReadEncM2
        ReadSpeedM1
        ReadSpeedM2
        ResetEncoders
        ReadPWMs
        ReadCurrents
        ReadTemp
        ReadTemp2
        ReadError
    """

    class Cmd:
        M1FORWARD = 0
        M1BACKWARD = 1
        SETMINMB = 2
        SETMAXMB = 3
        M2FORWARD = 4
        M2BACKWARD = 5
        M17BIT = 6
        M27BIT = 7
        MIXEDFORWARD = 8
        MIXEDBACKWARD = 9
        MIXEDRIGHT = 10
        MIXEDLEFT = 11
        MIXEDFB = 12
        MIXEDLR = 13

        GETM1ENC = 16
        GETM2ENC = 17
        GETM1SPEED = 18
        GETM2SPEED = 19
        RESETENC = 20
        GETVERSION = 21
        SETM1ENCCOUNT = 22
        SETM2ENCCOUNT = 23
        GETMBATT = 24
        GETLBATT = 25

        GETM1ISPEED = 30
        GETM2ISPEED = 31
        M1DUTY = 32
        M2DUTY = 33
        MIXEDDUTY = 34
        M1SPEED = 35
        M2SPEED = 36
        MIXEDSPEED = 37

        GETBUFFERS = 47
        GETPWMS = 48
        GETCURRENTS = 49

        GETTEMP = 82
        GETTEMP2 = 83
        GETERROR = 90

    def __init__(self, comport, rate, timeout=0.01, retries=3):
        self.comport = comport
        self.rate = rate
        self.timeout = timeout
        self._trystimeout = retries
        self._crc = 0
        self._port = None
        self.last_error = None

    # ============================================================
    # CONNECTION
    # ============================================================

    def Open(self):
        try:
            self._port = serial.Serial(
                port=self.comport,
                baudrate=self.rate,
                timeout=1,
                write_timeout=1,
                inter_byte_timeout=self.timeout
            )

            self.last_error = None
            return 1

        except Exception as error:
            self.last_error = error
            print(
                "Could not open RoboClaw port {}: {}".format(
                    self.comport,
                    error
                )
            )
            return 0

    def Close(self):
        try:
            if self._port is not None and self._port.is_open:
                self._port.close()

            return 1

        except Exception as error:
            self.last_error = error
            return 0

    def close(self):
        return self.Close()

    def _check_port(self):
        if self._port is None:
            raise RuntimeError("RoboClaw serial port has not been opened")

        if not self._port.is_open:
            raise RuntimeError("RoboClaw serial port is closed")

    # ============================================================
    # CRC
    # ============================================================

    def crc_clear(self):
        self._crc = 0

    def crc_update(self, data):
        data = int(data) & 0xFF

        self._crc ^= data << 8

        for _ in range(8):
            if self._crc & 0x8000:
                self._crc = (self._crc << 1) ^ 0x1021
            else:
                self._crc <<= 1

        self._crc &= 0xFFFF

    # ============================================================
    # LOW-LEVEL SERIAL FUNCTIONS
    # ============================================================

    def _flush_input(self):
        self._check_port()
        self._port.reset_input_buffer()

    def _sendcommand(self, address, command):
        self._check_port()

        address = int(address) & 0xFF
        command = int(command) & 0xFF

        self.crc_clear()

        self.crc_update(address)
        self._port.write(bytes([address]))

        self.crc_update(command)
        self._port.write(bytes([command]))

    def _writebyte(self, value):
        value = int(value) & 0xFF

        self.crc_update(value)
        self._port.write(bytes([value]))

    def _writeword(self, value):
        value = int(value) & 0xFFFF

        self._writebyte((value >> 8) & 0xFF)
        self._writebyte(value & 0xFF)

    def _writesword(self, value):
        value = int(value)

        if value < -32768 or value > 32767:
            raise ValueError(
                "Signed 16-bit value must be between -32768 and 32767"
            )

        self._writeword(value & 0xFFFF)

    def _writelong(self, value):
        value = int(value) & 0xFFFFFFFF

        self._writebyte((value >> 24) & 0xFF)
        self._writebyte((value >> 16) & 0xFF)
        self._writebyte((value >> 8) & 0xFF)
        self._writebyte(value & 0xFF)

    def _writeslong(self, value):
        value = int(value)

        if value < -2147483648 or value > 2147483647:
            raise ValueError(
                "Signed 32-bit value must be between "
                "-2147483648 and 2147483647"
            )

        self._writelong(value & 0xFFFFFFFF)

    def _readbyte(self):
        data = self._port.read(1)

        if len(data) != 1:
            return 0, 0

        value = data[0]
        self.crc_update(value)

        return 1, value

    def _readword(self):
        success_high, high = self._readbyte()

        if not success_high:
            return 0, 0

        success_low, low = self._readbyte()

        if not success_low:
            return 0, 0

        return 1, (high << 8) | low

    def _readlong(self):
        result = 0

        for _ in range(4):
            success, value = self._readbyte()

            if not success:
                return 0, 0

            result = (result << 8) | value

        return 1, result

    def _readslong(self):
        success, value = self._readlong()

        if not success:
            return 0, 0

        if value & 0x80000000:
            value -= 0x100000000

        return 1, value

    def _readchecksumword(self):
        data = self._port.read(2)

        if len(data) != 2:
            return 0, 0

        checksum = (data[0] << 8) | data[1]

        return 1, checksum

    def _writechecksum(self):
        #expected_ack = 0xFF

        self._writeword(self._crc & 0xFFFF)
        self._port.flush()

        #acknowledgement = self._port.read(1)

        #if len(acknowledgement) != 1:
        #    return False

        return True#acknowledgement[0] == expected_ack

    # ============================================================
    # GENERIC WRITE COMMANDS
    # ============================================================

    def _write0(self, address, command):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    def _write1(self, address, command, value):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)
                self._writebyte(value)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    def _write4(self, address, command, value):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)
                self._writelong(value)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    def _writeS2(self, address, command, value):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)
                self._writesword(value)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    def _writeS2S2(
        self,
        address,
        command,
        value1,
        value2
    ):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)

                self._writesword(value1)
                self._writesword(value2)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    def _writeS4(self, address, command, value):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)
                self._writeslong(value)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    def _writeS4S4(
        self,
        address,
        command,
        value1,
        value2
    ):
        for _ in range(self._trystimeout):
            try:
                self._sendcommand(address, command)

                self._writeslong(value1)
                self._writeslong(value2)

                if self._writechecksum():
                    return True

            except serial.SerialException as error:
                self.last_error = error

        return False

    # ============================================================
    # GENERIC READ COMMANDS
    # ============================================================

    def _read1(self, address, command):
        for _ in range(self._trystimeout):
            self._flush_input()
            self._sendcommand(address, command)

            success, value = self._readbyte()

            if not success:
                continue

            crc_success, received_crc = self._readchecksumword()

            if not crc_success:
                continue

            if (self._crc & 0xFFFF) == received_crc:
                return 1, value

        return 0, 0

    def _read2(self, address, command):
        for _ in range(self._trystimeout):
            self._flush_input()
            self._sendcommand(address, command)

            success, value = self._readword()

            if not success:
                continue

            crc_success, received_crc = self._readchecksumword()

            if not crc_success:
                continue

            if (self._crc & 0xFFFF) == received_crc:
                return 1, value

        return 0, 0

    def _read4(self, address, command):
        for _ in range(self._trystimeout):
            self._flush_input()
            self._sendcommand(address, command)

            success, value = self._readlong()

            if not success:
                continue

            crc_success, received_crc = self._readchecksumword()

            if not crc_success:
                continue

            if (self._crc & 0xFFFF) == received_crc:
                return 1, value

        return 0, 0

    def _read4_1(self, address, command):
        for _ in range(self._trystimeout):
            self._flush_input()
            self._sendcommand(address, command)

            success_value, value = self._readslong()

            if not success_value:
                continue

            success_status, status = self._readbyte()

            if not success_status:
                continue

            crc_success, received_crc = self._readchecksumword()

            if not crc_success:
                continue

            if (self._crc & 0xFFFF) == received_crc:
                return 1, value, status

        return 0, 0, 0

    # ============================================================
    # MOTOR COMMANDS
    # ============================================================

    def ForwardM1(self, address, value):
        return self._write1(
            address,
            self.Cmd.M1FORWARD,
            value
        )

    def BackwardM1(self, address, value):
        return self._write1(
            address,
            self.Cmd.M1BACKWARD,
            value
        )

    def ForwardM2(self, address, value):
        return self._write1(
            address,
            self.Cmd.M2FORWARD,
            value
        )

    def BackwardM2(self, address, value):
        return self._write1(
            address,
            self.Cmd.M2BACKWARD,
            value
        )

    def ForwardBackwardM1(self, address, value):
        return self._write1(
            address,
            self.Cmd.M17BIT,
            value
        )

    def ForwardBackwardM2(self, address, value):
        return self._write1(
            address,
            self.Cmd.M27BIT,
            value
        )

    def DutyM1(self, address, duty):
        return self._writeS2(
            address,
            self.Cmd.M1DUTY,
            duty
        )

    def DutyM2(self, address, duty):
        return self._writeS2(
            address,
            self.Cmd.M2DUTY,
            duty
        )

    def DutyM1M2(self, address, m1_duty, m2_duty):
        return self._writeS2S2(
            address,
            self.Cmd.MIXEDDUTY,
            m1_duty,
            m2_duty
        )

    def SpeedM1(self, address, speed):
        return self._writeS4(
            address,
            self.Cmd.M1SPEED,
            speed
        )

    def SpeedM2(self, address, speed):
        return self._writeS4(
            address,
            self.Cmd.M2SPEED,
            speed
        )

    def SpeedM1M2(self, address, m1_speed, m2_speed):
        return self._writeS4S4(
            address,
            self.Cmd.MIXEDSPEED,
            m1_speed,
            m2_speed
        )

    # ============================================================
    # ENCODERS AND SPEED
    # ============================================================

    def ReadEncM1(self, address):
        return self._read4_1(
            address,
            self.Cmd.GETM1ENC
        )

    def ReadEncM2(self, address):
        return self._read4_1(
            address,
            self.Cmd.GETM2ENC
        )

    def ReadSpeedM1(self, address):
        return self._read4_1(
            address,
            self.Cmd.GETM1SPEED
        )

    def ReadSpeedM2(self, address):
        return self._read4_1(
            address,
            self.Cmd.GETM2SPEED
        )

    def ReadISpeedM1(self, address):
        return self._read4_1(
            address,
            self.Cmd.GETM1ISPEED
        )

    def ReadISpeedM2(self, address):
        return self._read4_1(
            address,
            self.Cmd.GETM2ISPEED
        )

    def ResetEncoders(self, address):
        return self._write0(
            address,
            self.Cmd.RESETENC
        )

    def SetEncM1(self, address, count):
        return self._write4(
            address,
            self.Cmd.SETM1ENCCOUNT,
            count
        )

    def SetEncM2(self, address, count):
        return self._write4(
            address,
            self.Cmd.SETM2ENCCOUNT,
            count
        )

    # ============================================================
    # VERSION
    # ============================================================

    def ReadVersion(self, address):
        for _ in range(self._trystimeout):
            self._flush_input()
            self._sendcommand(
                address,
                self.Cmd.GETVERSION
            )

            version_bytes = bytearray()
            complete = False

            for _ in range(48):
                data = self._port.read(1)

                if len(data) != 1:
                    break

                value = data[0]
                self.crc_update(value)

                if value == 0:
                    complete = True
                    break

                version_bytes.append(value)

            if not complete:
                continue

            crc_success, received_crc = self._readchecksumword()

            if not crc_success:
                continue

            if (self._crc & 0xFFFF) != received_crc:
                continue

            version = version_bytes.decode(
                "ascii",
                errors="replace"
            )

            return 1, version

        return 0, ""

    # ============================================================
    # BATTERY VOLTAGES
    # ============================================================

    def ReadMainBatteryVoltage(self, address):
        return self._read2(
            address,
            self.Cmd.GETMBATT
        )

    def ReadLogicBatteryVoltage(self, address):
        return self._read2(
            address,
            self.Cmd.GETLBATT
        )

    # ============================================================
    # PWM AND CURRENT
    # ============================================================

    def ReadPWMs(self, address):
        success, value = self._read4(
            address,
            self.Cmd.GETPWMS
        )

        if not success:
            return 0, 0, 0

        pwm1 = (value >> 16) & 0xFFFF
        pwm2 = value & 0xFFFF

        if pwm1 & 0x8000:
            pwm1 -= 0x10000

        if pwm2 & 0x8000:
            pwm2 -= 0x10000

        return 1, pwm1, pwm2

    def ReadCurrents(self, address):
        success, value = self._read4(
            address,
            self.Cmd.GETCURRENTS
        )

        if not success:
            return 0, 0, 0

        current1 = (value >> 16) & 0xFFFF
        current2 = value & 0xFFFF

        return 1, current1, current2

    # ============================================================
    # TEMPERATURE AND ERRORS
    # ============================================================

    def ReadTemp(self, address):
        return self._read2(
            address,
            self.Cmd.GETTEMP
        )

    def ReadTemp2(self, address):
        return self._read2(
            address,
            self.Cmd.GETTEMP2
        )

    def ReadError(self, address):
        return self._read4(
            address,
            self.Cmd.GETERROR
        )

    # ============================================================
    # UTILITY
    # ============================================================

    def SendRandomData(self, count):
        self._check_port()

        for _ in range(count):
            value = random.getrandbits(8)
            self._port.write(bytes([value]))
