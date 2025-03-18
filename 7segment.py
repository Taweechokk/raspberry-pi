import lgpio
from time import sleep

# Define GPIO pins
CLK = 5  # GPIO Pin for CLK
DIO = 6  # GPIO Pin for DIO

# TM1637 Commands
TM1637_CMD1 = 0x40  # Command 1: Data command setting
TM1637_CMD2 = 0xC0  # Command 2: Address command setting
TM1637_CMD3 = 0x80  # Command 3: Display control command
TM1637_DSP_ON = 0x08  # Display ON flag
TM1637_DELAY = 0.00001  # Small delay for timing

# Segment definitions for 0-9, A-Z, blank, dash, star
_SEGMENTS = bytearray(
    b'\x3F\x06\x5B\x4F\x66\x6D\x7D\x07\x7F\x6F\x77\x7C\x39\x5E\x79\x71'
    b'\x3D\x76\x06\x1E\x76\x38\x55\x54\x3F\x73\x67\x50\x6D\x78\x3E\x1C\x2A\x76\x6E\x5B\x00\x40\x63'
)


class TM1637:
    """Driver for TM1637 7-segment display using lgpio"""

    def __init__(self, clk, dio, brightness=7):
        self.clk = clk
        self.dio = dio
        self.handle = lgpio.gpiochip_open(0)  # Open the GPIO chip
        lgpio.gpio_claim_output(self.handle, self.clk)
        lgpio.gpio_claim_output(self.handle, self.dio)
        self.brightness = brightness
        self._write_data_cmd()
        self._write_dsp_ctrl()

    def _start(self):
        lgpio.gpio_write(self.handle, self.dio, 1)
        lgpio.gpio_write(self.handle, self.clk, 1)
        lgpio.gpio_write(self.handle, self.dio, 0)
        lgpio.gpio_write(self.handle, self.clk, 0)

    def _stop(self):
        lgpio.gpio_write(self.handle, self.clk, 0)
        lgpio.gpio_write(self.handle, self.dio, 0)
        lgpio.gpio_write(self.handle, self.clk, 1)
        lgpio.gpio_write(self.handle, self.dio, 1)

    def _write_data_cmd(self):
        self._start()
        self._write_byte(TM1637_CMD1)
        self._stop()
        
    def _write_dsp_ctrl(self):
        self._start()
        self._write_byte(TM1637_CMD3 | TM1637_DSP_ON | self.brightness)
        self._stop()

    def _write_byte(self, data):
        for i in range(8):
            lgpio.gpio_write(self.handle, self.dio, (data >> i) & 1)
            sleep(TM1637_DELAY)
            lgpio.gpio_write(self.handle, self.clk, 1)
            sleep(TM1637_DELAY)
            lgpio.gpio_write(self.handle, self.clk, 0)
            sleep(TM1637_DELAY)

        lgpio.gpio_write(self.handle, self.clk, 0)
        sleep(TM1637_DELAY)
        lgpio.gpio_write(self.handle, self.clk, 1)
        sleep(TM1637_DELAY)
        lgpio.gpio_write(self.handle, self.clk, 0)

    def show(self, text):
        segments = self.encode_string(text)
        self._write_data_cmd()
        self._start()
        self._write_byte(TM1637_CMD2)
        for seg in segments:
            self._write_byte(seg)
        self._stop()
        self._write_dsp_ctrl()

    def encode_string(self, string):
        segments = bytearray(len(string))
        for i, char in enumerate(string):
            segments[i] = self.encode_char(char)
        return segments

    @staticmethod
    def encode_char(char):
        o = ord(char)
        if 48 <= o <= 57:  # 0-9
            return _SEGMENTS[o - 48]
        elif 65 <= o <= 90:  # A-Z
            return _SEGMENTS[o - 55]
        elif 97 <= o <= 122:  # a-z
            return _SEGMENTS[o - 87]
        elif o == 32:  # Space
            return _SEGMENTS[36]
        elif o == 45:  # Dash
            return _SEGMENTS[37]
        return 0  # Default blank

    def cleanup(self):
        lgpio.gpiochip_close(self.handle)

    def clear(self):
        """Turn off all segments of the display"""
        self._write_data_cmd()
        self._start()
        self._write_byte(TM1637_CMD2)
        for _ in range(4):  # 4 digits in the display
            self._write_byte(0)  # Write 0 to turn off all segments
        self._stop()
        self._write_dsp_ctrl()


# ใช้งาน TM1637
tm = TM1637(CLK, DIO)
tm.show("1234")
sleep(2)
tm.show("5678")
sleep(2)
tm.show("0123")
sleep(2)
tm.show("4569")
sleep(2)
tm.clear()
tm.cleanup()
