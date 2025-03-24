import serial
import gpiozero.pins.lgpio
import lgpio
from time import sleep
from gpiozero import *
import board
import adafruit_dht

# ตั้งค่าพอร์ตอนุกรมเสมือน
SERIAL_PORT = "/dev/pts/5"  # ใช้ Virtual Serial Port ที่สร้างจาก tty0tty
BAUD_RATE = 9600

# ser = serial.Serial('/dev/pts/5', baudrate=9600, timeout=1)

# คำสั่งที่รองรับ
COMMANDS = {
    # พื้นฐาน GPIO
    'dr': 'read_digital',
    'ar': 'read_analog',
    'dw': 'digitalWrite',
    'aw': 'write_analog',
    'itr': 'set_interrupt',

    # Built-in
    'k_bn': 'kb_button',
    'k_tm': 'kb_temperature',
    'k_mt': 'kb_matrix_text',
    'k_mp': 'kb_matrix_pattern',
    'k_ld': 'kb_led',
    'k_gy': 'kb_gyro',

    # Sensors
    'ultra': 'read_ultrasonic',
    'b280': 'read_bme280',
    'd11': 'read_dht11',
    'ds18': 'read_ds18b20',
    'mic': 'read_microphone',
    'mpu': 'read_mpu6050',

    # Actuators
    'buz': 'play_buzzer',
    'srv': 'move_servo',
    'lcd': 'write_lcd',
    'sg': 'write_7segment',
    'rgb': 'write_rgb_led'
}


# init
def __patched_init(self, chip=None):
    gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpiozero.pins.lgpio.LGPIOPin


class TM1637:
    """Driver for TM1637 7-segment display using lgpio"""

    # ---- Class Constants ----
    CMD1 = 0x40  # Data command setting
    CMD2 = 0xC0  # Address command setting
    CMD3 = 0x80  # Display control command
    DSP_ON = 0x08
    DELAY = 0.00001

    SEGMENTS = bytearray(
        b'\x3F\x06\x5B\x4F\x66\x6D\x7D\x07\x7F\x6F\x77\x7C\x39\x5E\x79\x71'
        b'\x3D\x76\x06\x1E\x76\x38\x55\x54\x3F\x73\x67\x50\x6D\x78\x3E\x1C\x2A\x76\x6E\x5B\x00\x40\x63'
    )

    def __init__(self, clk, dio, brightness=7):
        self.clk = clk
        self.dio = dio
        self.handle = lgpio.gpiochip_open(0)
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
        self._write_byte(self.CMD1)
        self._stop()

    def _write_dsp_ctrl(self):
        self._start()
        self._write_byte(self.CMD3 | self.DSP_ON | self.brightness)
        self._stop()

    def _write_byte(self, data):
        for i in range(8):
            lgpio.gpio_write(self.handle, self.dio, (data >> i) & 1)
            sleep(self.DELAY)
            lgpio.gpio_write(self.handle, self.clk, 1)
            sleep(self.DELAY)
            lgpio.gpio_write(self.handle, self.clk, 0)
            sleep(self.DELAY)

        lgpio.gpio_write(self.handle, self.clk, 0)
        sleep(self.DELAY)
        lgpio.gpio_write(self.handle, self.clk, 1)
        sleep(self.DELAY)
        lgpio.gpio_write(self.handle, self.clk, 0)

    def show(self, text):
        text = str(text)
        segments = self.encode_string(text)

        # 👇 เติมช่องว่าง (0x00) ทางซ้ายให้ครบ 4 หลัก
        while len(segments) < 4:
            segments.insert(0, 0x00)  # เพิ่ม segment ว่างซ้ายสุด

        self._write_data_cmd()
        self._start()
        self._write_byte(self.CMD2)
        for seg in segments:
            self._write_byte(seg)
        self._stop()
        self._write_dsp_ctrl()
        

    def encode_string(self, string):
        segments = bytearray(len(string))
        for i, char in enumerate(string):
            segments[i] = self.encode_char(char)
        return segments

    @classmethod
    def encode_char(cls, char):
        o = ord(char)
        if 48 <= o <= 57:  # 0-9
            return cls.SEGMENTS[o - 48]
        elif 65 <= o <= 90:  # A-Z
            return cls.SEGMENTS[o - 55]
        elif 97 <= o <= 122:  # a-z
            return cls.SEGMENTS[o - 87]
        elif o == 32:  # Space
            return cls.SEGMENTS[36]
        elif o == 45:  # Dash
            return cls.SEGMENTS[37]
        return 0

    def clear(self):
        self._write_data_cmd()
        self._start()
        self._write_byte(self.CMD2)
        for _ in range(4):
            self._write_byte(0)
        self._stop()
        self._write_dsp_ctrl()

    def cleanup(self):
        lgpio.gpiochip_close(self.handle)

def read_dht11(pin_number,mode):
    pin_obj = getattr(board, f"D{pin_number}")  # แปลงเป็น board.D26
    dht = adafruit_dht.DHT11(pin_obj)
    if mode == "T":
        temp = dht.temperature
        dht.exit()
        return temp
    elif mode == "H":
        humidity = dht.humidity
        dht.exit()
        return humidity
    else:
        dht.exit()
        return None

# แปลงคำสั่ง
def parse_command(command_str):
    try:
        command_str = command_str.strip()
        parts = command_str.split(":")

        if len(parts) < 2:
            return None, None, None

        cmd_name, cmd_id = parts[:2]
        cmd_args = parts[2:] if len(parts) > 2 else []

        if cmd_name not in COMMANDS:
            return None, cmd_id, None

        return cmd_name, cmd_id, cmd_args

    except Exception as e:
        print(f"Error parsing command: {e}")
        return None, None, None

# ประมวลผลคำสั่ง
def execute_command(cmd_name, cmd_id, cmd_args):
    success = 0
    try:

        if cmd_name == 'dw':  # เขียน Digital
            if len(cmd_args) == 2:                                                                                                                                                                                                                                                                   
                dwpin, dwvalue = int(cmd_args[0]), int(cmd_args[1])
                digitalWrite = DigitalOutputDevice(pin=dwpin)
                if dwvalue == 1:
                    digitalWrite.on()
                elif dwvalue == 0:
                    digitalWrite.off()
                else:
                    print("Invalid value")
                success = 1

        elif cmd_name == 'sg':  # display number on 7-segment
            if len(cmd_args) == 3:
                clk, dio, number = int(cmd_args[0]), int(cmd_args[1]), (cmd_args[2])
                tm = TM1637(clk, dio)
                if number == "end":
                    tm.clear()
                    tm.cleanup()
                else:
                    tm.show(number)
                    tm.cleanup()
                success = 1

        elif cmd_name == 'd11':
            if len(cmd_args) == 2:
                pin, mode = int(cmd_args[0]), cmd_args[1]
                result = read_dht11(pin, mode)
                print(f"Result: {result}")
                success = 1

        

    except Exception as e:
        print(f"Execution error: {e}")

    return f"{cmd_name}:{cmd_id}:{success}"

# ฟังก์ชันหลัก
def main():
    try:
        gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")

        while True:
            try:
                
                if ser.in_waiting > 0:  # เช็คว่ามีข้อมูลรอรับหรือไม่
                    raw_command = ser.readline().decode('utf-8').strip()  # อ่านคำสั่งและตัดช่องว่าง
                    print(f"Received: {raw_command}")  # Debug log
                    
                    if raw_command:
                        cmd_name, cmd_id, cmd_args = parse_command(raw_command)
                        if cmd_name:
                            response = execute_command(cmd_name, cmd_id, cmd_args)
                        else:
                            response = f"error:{cmd_id}:0"

                        ser.write((response + "\n").encode('utf-8'))  # ส่งผลลัพธ์กลับ
                        print(f"Sent: {response}")

                    ser.reset_input_buffer()  # ล้างบัฟเฟอร์ Input
                    

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        # lgpio.gpiochip_close(HANDLE)

if __name__ == "__main__":
    main()
