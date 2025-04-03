# Complete Project Details: https://RandomNerdTutorials.com/raspberry-pi-analog-inputs-python-mcp3008/

from gpiozero import PWMLED, MCP3008
from time import sleep
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

#create an object called pot that refers to MCP3008 channel 0
pot = MCP3008(0)



while True:
    print("Potentiometer value: ", pot.value) # print the potentiometer value

    sleep(0.5)