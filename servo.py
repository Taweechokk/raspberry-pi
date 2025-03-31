# from gpiozero import *
# from time import sleep
# import gpiozero.pins.lgpio
# import lgpio

# # init
# def __patched_init(self, chip=None):
#     gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
#     chip = 0
#     self._handle = lgpio.gpiochip_open(chip)
#     self._chip = chip
#     self.pin_class = gpiozero.pins.lgpio.LGPIOPin

# gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init
# # กำหนดขา GPIO ที่เชื่อมกับสายสัญญาณของ Servo
# servo = PWMOutputDevice(pin=19, frequency=50)  # ใช้ PWM ที่ 50Hz

# # ฟังก์ชันแปลงองศา (0-180) เป็น duty cycle (ประมาณ 0.03 - 0.12 สำหรับ 0-180 องศา)
# def angle_to_duty(angle):
#     # ปรับค่าตาม servo ที่ใช้ โดยทั่วไป 0.5ms ถึง 2.5ms => 2.5% ถึง 12.5% ที่ 50Hz
#     min_duty = 0.03  # 3%
#     max_duty = 0.12  # 12%
#     duty = min_duty + (angle / 180.0) * (max_duty - min_duty)
#     return duty

# # ฟังก์ชันหมุน servo ไปยังองศาที่ต้องการ
# def set_servo_angle(angle):
#     duty = angle_to_duty(angle)
#     print(f"Setting angle to {angle}° → Duty cycle: {duty:.4f}")
#     servo.value = duty
#     sleep(0.5)  # รอให้ servo ขยับเสร็จ

# # ทดสอบหมุน servo
# try:
#     for angle in range(0, 181, 30):  # หมุนทีละ 30 องศา
#         set_servo_angle(angle)
#     sleep(1)
#     for angle in reversed(range(0, 181, 30)):
#         set_servo_angle(angle)

# finally:
#     print("Stop servo signal")
#     servo.value = 0  # ปิด PWM


# from gpiozero import AngularServo

# from time import sleep

# Create an AngularServo object with the specified GPIO pin,

# minimum pulse width, and maximum pulse width

# servo = AngularServo(19, min_pulse_width=0.0006, max_pulse_width=0.0023)
# servo = AngularServo(19,
#                      min_angle=-90,
#                      max_angle=90,
#                      min_pulse_width=0.0006,
#                      max_pulse_width=0.0024)

# try:

#    while True:

#        # Set the servo angle to 90 degrees

#        servo.angle = 90

#        sleep(2)  # Delay for 1 second

#        # Set the servo angle to 0 degrees

#        servo.angle = 0

#        sleep(2)  # Delay for 1 second

#        # Set the servo angle to -90 degrees

#        servo.angle = -90

#        sleep(1)  # Delay for 1 second

# finally:

#    # Set the servo angle to 0 degrees before exiting

#    servo.angle = 0

from gpiozero import AngularServo
from time import sleep
import gpiozero.pins.lgpio
import lgpio

# Patch เพื่อให้ใช้ LGPIO ได้
def __patched_init(self, chip=None):
    gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpiozero.pins.lgpio.LGPIOPin

gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init

# สร้าง servo object ด้วยค่าที่เหมาะสมกับ SG92R
servo = AngularServo(13,
                     min_angle=0,
                     max_angle=180,
                     min_pulse_width=0.0005,
                     max_pulse_width=0.0025)



servo.angle = 0
sleep(2)
servo.angle = 90
sleep(2)
servo.angle = 180
sleep(2)
servo.angle = 0
sleep(2)



