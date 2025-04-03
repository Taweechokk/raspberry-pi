import serial
import gpiozero.pins.lgpio
import lgpio
from time import sleep
from gpiozero import *
import board
import adafruit_dht
import smbus
import bme280
import os
import glob
from gpiozero.tones import Tone


# ตั้งค่าพอร์ตอนุกรมเสมือน
SERIAL_PORT = "/dev/pts/5"  # ใช้ Virtual Serial Port ที่สร้างจาก tty0tty
BAUD_RATE = 115200

# ser = serial.Serial('/dev/pts/5', baudrate=9600, timeout=1)

# คำสั่งที่รองรับ
COMMANDS = {
    # พื้นฐาน GPIO
    'dr': 'read_digital',
    'ar': 'read_analog',
    'dw': 'digitalWrite',
    'aw': 'write_analog',
    'itr': 'set_interrupt',

    # Sensors
    'ultra': 'read_ultrasonic',
    'b280': 'read_bme280',
    'd11': 'read_dht11',
    'ds18': 'read_ds18b20',
    'mic': 'read_microphone',

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

gpio_devices = {}

def digitalWrite(pin, value):
    global gpio_devices
    if pin not in gpio_devices or not isinstance(gpio_devices[pin], DigitalOutputDevice):
        gpio_devices[pin] = DigitalOutputDevice(pin=pin)
        
    digitalDevice = gpio_devices[pin]

    if value == 1:
        digitalDevice.on()
        print(f"✅ GPIO{pin} ON")
    elif value == 0:
        digitalDevice.off()
        digitalDevice.close()        # ✅ ปิดอุปกรณ์
        del gpio_devices[pin]         # ✅ ลบออกจาก dict
        print(f"🛑 GPIO{pin} OFF & released")
    else:
        print("Invalid value")

def analogWrite(pin, freq, duty):
    global gpio_devices

    # ถ้า freq หรือ duty เป็น 0 → ปิดและลบอุปกรณ์
    if freq == 0 or duty == 0:
        if pin in gpio_devices:
            gpio_devices[pin].close()
            del gpio_devices[pin]
            print(f"🛑 ปิดและคืน GPIO{pin} เพราะ freq หรือ duty = 0")
        else:
            print(f"⚠️ GPIO{pin} ยังไม่ถูกใช้งาน ไม่มีอุปกรณ์ให้ปิด")
        return  # ไม่ต้องทำงานต่อ

    # ถ้ายังไม่เคยสร้าง PWM บน pin นี้
    if pin not in gpio_devices or not isinstance(gpio_devices[pin], PWMOutputDevice):
        gpio_devices[pin] = PWMOutputDevice(pin=pin, frequency=freq, initial_value=duty)
        print(f"🔧 สร้าง PWMOutputDevice(pin={pin}, freq={freq}, duty={duty})")
    else:
        pwm_device = gpio_devices[pin]
        if pwm_device.frequency != freq:
            pwm_device.frequency = freq
            print(f"🎚 ปรับ frequency GPIO{pin} → {freq}Hz")
        pwm_device.value = duty
        print(f"⚡ ปรับ duty GPIO{pin} → {duty*100:.1f}%")

def digitalRead(pin):
    if pin not in gpio_devices or not isinstance(gpio_devices[pin], DigitalInputDevice):
        gpio_devices[pin] = DigitalInputDevice(pin=pin, pull_up=False)
        print(f"📥 สร้าง DigitalInputDevice(GPIO{pin})")

    value = gpio_devices[pin].value
    gpio_devices[pin].close()
    del gpio_devices[pin]
    print(f"📌 GPIO{pin} = {value}")
    return value

def setupRisingInterrupt(pin, mode):
    global gpio_devices

    if mode == "attach":
        # สร้างอุปกรณ์ ถ้ายังไม่มี หรือชนิดไม่ตรง
        if pin not in gpio_devices or not isinstance(gpio_devices[pin], DigitalInputDevice):
            gpio_devices[pin] = DigitalInputDevice(pin=pin, pull_up=False)
            print(f"🔗 สร้าง DigitalInputDevice(GPIO{pin}) สำหรับ interrupt")

        device = gpio_devices[pin]

        # สร้างฟังก์ชัน interrupt ด้านใน
        def on_rising():
            print(f"🚨 ตรวจพบขอบขาขึ้นที่ GPIO{pin}")

        device.when_activated = on_rising
        print(f"✅ ติดตั้ง interrupt (ขอบขาขึ้น) สำเร็จที่ GPIO{pin}")

    elif mode == "detach":
        if pin in gpio_devices and isinstance(gpio_devices[pin], DigitalInputDevice):
            gpio_devices[pin].when_activated = None
            gpio_devices[pin].close()
            del gpio_devices[pin]
            print(f"🧹 ยกเลิก interrupt และคืน GPIO{pin} เรียบร้อยแล้ว")
        else:
            print(f"⚠️ ไม่พบอุปกรณ์ DigitalInput บน GPIO{pin} เพื่อถอด")

    else:
        print("❌ mode ต้องเป็น 'attach' หรือ 'detach' เท่านั้น")

def analogRead(channel):
    adc = MCP3008(channel=channel)
    voltage = round(adc.value *5, 2)        # แปลงเป็นแรงดันไฟฟ้า (0-5V)
    adc.close()
    return voltage

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

def ultrasonic(trig, echo):
    sensor = DistanceSensor(trigger=trig, echo=echo, max_distance=2)
    distance = sensor.distance * 100
    sensor.close()
    return distance

class LCD:
    def __init__(self, pi_rev , i2c_addr, backlight = True):

        # device constants
        self.I2C_ADDR  = i2c_addr
        self.LCD_WIDTH = 16   # Max. characters per line

        self.LCD_CHR = 1 # Mode - Sending data
        self.LCD_CMD = 0 # Mode - Sending command

        self.LCD_LINE_1 = 0x80 # LCD RAM addr for line one
        self.LCD_LINE_2 = 0xC0 # LCD RAM addr for line two

        if backlight:
            # on
            self.LCD_BACKLIGHT  = 0x08
        else:
            # off
            self.LCD_BACKLIGHT = 0x00

        self.ENABLE = 0b00000100 # Enable bit

        # Timing constants
        self.E_PULSE = 0.0005
        self.E_DELAY = 0.0005

        # Open I2C interface
        if pi_rev == 2:
            # Rev 2 Pi uses 1
            self.bus = smbus.SMBus(1)
        elif pi_rev == 1:
            # Rev 1 Pi uses 0
            self.bus = smbus.SMBus(0)
        else:
            raise ValueError('pi_rev param must be 1 or 2')

        # Initialise display
        self.lcd_byte(0x33, self.LCD_CMD) # 110011 Initialise
        self.lcd_byte(0x32, self.LCD_CMD) # 110010 Initialise
        self.lcd_byte(0x06, self.LCD_CMD) # 000110 Cursor move direction
        self.lcd_byte(0x0C, self.LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
        self.lcd_byte(0x28, self.LCD_CMD) # 101000 Data length, number of lines, font size
        self.lcd_byte(0x01, self.LCD_CMD) # 000001 Clear display

    def lcd_byte(self, bits, mode):
        # Send byte to data pins
        # bits = data
        # mode = 1 for data, 0 for command

        bits_high = mode | (bits & 0xF0) | self.LCD_BACKLIGHT
        bits_low = mode | ((bits<<4) & 0xF0) | self.LCD_BACKLIGHT

        # High bits
        self.bus.write_byte(self.I2C_ADDR, bits_high)
        self.toggle_enable(bits_high)

        # Low bits
        self.bus.write_byte(self.I2C_ADDR, bits_low)
        self.toggle_enable(bits_low)

    def toggle_enable(self, bits):
        sleep(self.E_DELAY)
        self.bus.write_byte(self.I2C_ADDR, (bits | self.ENABLE))
        sleep(self.E_PULSE)
        self.bus.write_byte(self.I2C_ADDR,(bits & ~self.ENABLE))
        sleep(self.E_DELAY)

    def message(self, string, line = 1):
        # display message string on LCD line 1 or 2
        if line == 1:
            lcd_line = self.LCD_LINE_1
        elif line == 2:
            lcd_line = self.LCD_LINE_2
        else:
            raise ValueError('line number must be 1 or 2')

        string = string.ljust(self.LCD_WIDTH," ")

        self.lcd_byte(lcd_line, self.LCD_CMD)

        for i in range(self.LCD_WIDTH):
            self.lcd_byte(ord(string[i]), self.LCD_CHR)

    def clear(self):
        # clear LCD display
        self.lcd_byte(0x01, self.LCD_CMD)

def read_bme280(address, mode):
    bus = smbus.SMBus(1)
    calibration_params = bme280.load_calibration_params(bus, address)
    data = bme280.sample(bus, address, calibration_params)

    if mode == "T":
        return round(data.temperature, 2)
    elif mode == "P":
        return round(data.pressure, 2)
    elif mode == "H":
        return round(data.humidity, 2)
    elif mode == "A":
        pressure = data.pressure
        altitude = 44330 * (1 - (pressure / 1013.25) ** (1/5.255))
        return round(altitude, 2)
    else:
        return None

def ds18b20():
    # เปิดใช้งาน 1-Wire Interface (กรณีไม่ได้เปิดไว้แล้ว)
        print("Loading 1-Wire kernel modules...")
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')

        # ค้นหา device
        base_dir = '/sys/bus/w1/devices/'
        device_folders = glob.glob(base_dir + '28*')
        if not device_folders:
            return {"error": "No DS18B20 device found"}
        
        device_file = device_folders[0] + '/w1_slave'

        # อ่านข้อมูลจากไฟล์
        with open(device_file, 'r') as f:
            lines = f.readlines()

        while lines[0].strip()[-3:] != 'YES':
            sleep(0.2)
            with open(device_file, 'r') as f:
                lines = f.readlines()

        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0
            return round(temp_c, 2)

        return {"error": "Invalid temperature format"}

def LEDrgb(pin_red, pin_green, pin_blue, val_red, val_green, val_blue):
    """
    ฟังก์ชันควบคุม RGB LED แบบ PWM พร้อมบันทึกไว้ใน gpio_devices
    หากค่า R, G, B = 0 ทั้งหมด จะปิด LED และคืนพิน
    
    Parameters:
        pin_red (int): พินสีแดง
        pin_green (int): พินสีเขียว
        pin_blue (int): พินสีน้ำเงิน
        val_red (float): ความสว่างสีแดง (0.0 - 1.0)
        val_green (float): ความสว่างสีเขียว (0.0 - 1.0)
        val_blue (float): ความสว่างสีน้ำเงิน (0.0 - 1.0)
    """
    key = f"rgb_{pin_red}_{pin_green}_{pin_blue}"

    # หากมีอุปกรณ์นี้อยู่แล้ว ให้ดึงมาใช้ซ้ำ
    if key in gpio_devices:
        led = gpio_devices[key]
    else:
        led = RGBLED(pin_red, pin_green, pin_blue, pwm=True)
        gpio_devices[key] = led

    # ถ้าค่าทุกสีเป็น 0 → ปิดและคืนพิน
    if val_red == 0.0 and val_green == 0.0 and val_blue == 0.0:
        led.off()
        led.close()
        del gpio_devices[key]
        print(f"LED RGB at {key} turned OFF and pin released")
    else:
        led.color = (val_red, val_green, val_blue)
        print(f"LED RGB at {key} set to color = ({val_red}, {val_green}, {val_blue})")

def play_tonal_buzzer(pin: int, tone_hz: float):
    """
    ฟังก์ชันเล่นเสียงด้วย TonalBuzzer ที่สามารถรับ tone แบบ Hz
    - หาก tone_hz = 0 → ปิดเสียงและคืนพิน

    Parameters:
        pin (int): พินที่ต่อกับ Buzzer
        tone_hz (float): ความถี่เสียงเป็น Hz (เช่น 440.0), ถ้า 0 คือหยุด
    """
    key = f"tonal_buzzer_{pin}"

    # หาก tone_hz = 0 → ปิดและคืนพิน
    if tone_hz == 0:
        if key in gpio_devices:
            buzzer = gpio_devices[key]
            buzzer.stop()
            buzzer.close()
            del gpio_devices[key]
            print(f"Tonal Buzzer on pin {pin} stopped and released.")
        else:
            print(f"No active buzzer on pin {pin} to stop.")
        return

    # เริ่มเล่นเสียง
    if key not in gpio_devices:
        buzzer = TonalBuzzer(pin)
        gpio_devices[key] = buzzer
    else:
        buzzer = gpio_devices[key]

    try:
        buzzer.play(Tone(tone_hz))
        print(f"Playing tone {tone_hz} Hz on pin {pin}")
    except Exception as e:
        print(f"Error: {e}")

def servo(pin, angle):
    servo = AngularServo(pin,
                     min_angle=0,
                     max_angle=180,
                     min_pulse_width=0.0005,
                     max_pulse_width=0.0025)
    servo.angle = angle
    sleep(0.4)  # รอให้ servo ขยับเสร็จ
    servo.close()

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

        if cmd_name == 'dw':  # digital write
            if len(cmd_args) == 2:                                                                                                                                                                                                                                                                   
                dwpin, dwvalue = int(cmd_args[0]), int(cmd_args[1])
                digitalWrite(dwpin, dwvalue)
                success = 1

        elif cmd_name == 'ar':  # analog read
            if len(cmd_args) == 1:
                ar_channel = int(cmd_args[0])
                ar_value = analogRead(ar_channel)
                print(f"Value: {ar_value}")
                success = 1

        elif cmd_name == 'aw':  # analog write
            if len(cmd_args) == 3:
                awpin, awfreq, awduty = int(cmd_args[0]), int(cmd_args[1]), float(cmd_args[2])
                analogWrite(awpin, awfreq, awduty)
                success = 1

        elif cmd_name == 'dr':  # digital read
            if len(cmd_args) == 1:
                drpin = int(cmd_args[0])
                dr_value = digitalRead(drpin)
                print(f"Value: {dr_value}")
                success = 1

        elif cmd_name == 'itr':  # set interrupt pin on rising edge
            if len(cmd_args) == 2:
                pin,mode = int(cmd_args[0]), cmd_args[1]
                setupRisingInterrupt(pin, mode)
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

        elif cmd_name == 'd11': # read DHT11 sensor
            if len(cmd_args) == 2:
                pin, mode = int(cmd_args[0]), cmd_args[1]
                result = read_dht11(pin, mode)
                print(f"Result: {result}")
                success = 1

        elif cmd_name == 'ultra':   # read ultrasonic sensor
            if len(cmd_args) == 2:
                trig, echo = int(cmd_args[0]), int(cmd_args[1])
                distance = ultrasonic(trig, echo)
                print(f"Distance: {distance}")
                success = 1

        elif cmd_name == 'srv': # move servo
            if len(cmd_args) == 2:
                pin, angle = int(cmd_args[0]), int(cmd_args[1])
                servo(pin, angle)
                success = 1

        elif cmd_name == 'lcd': # write to LCD
            print({len(cmd_args)})
            if len(cmd_args) == 3:
                i2c_addr, text1, text2 = int(cmd_args[0],0), cmd_args[1], cmd_args[2]
                print(f"i2c_addr: {i2c_addr}, text1: {text1}, text2: {text2}")
                lcd = LCD(2, i2c_addr, True)
                print(f"LCD: {text1}, {text2}")
                lcd.message(text1, 1)
                lcd.message(text2, 2)
                success = 1

        elif cmd_name == 'b280': # read BME280 sensor
            if len(cmd_args) == 2:
                address, mode = int(cmd_args[0],0), cmd_args[1]
                print(f"Address: {address}, Mode: {mode}")
                result = read_bme280(address, mode)
                print(f"Result: {result}")
                success = 1

        elif cmd_name == 'ds18': # read DS18B20 sensor
            if len(cmd_args) == 0:
                result = ds18b20()
                print(f"Result: {result}")
                success = 1

        elif cmd_name == 'rgb':  # write to RGB LED
            if len(cmd_args) == 6:
                pin_red, pin_green, pin_blue = int(cmd_args[0]), int(cmd_args[1]), int(cmd_args[2])
                val_red, val_green, val_blue = float(cmd_args[3]), float(cmd_args[4]), float(cmd_args[5])
                LEDrgb(pin_red, pin_green, pin_blue, val_red, val_green, val_blue)
                success = 1

        elif cmd_name == 'buz':  # play buzzer
            if len(cmd_args) == 2:
                pin, tone_hz = int(cmd_args[0]), float(cmd_args[1])
                play_tonal_buzzer(pin, tone_hz)
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
