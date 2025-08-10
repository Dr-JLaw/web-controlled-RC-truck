


from gpiozero import Servo, Button, LED
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
led=LED(26)
PS=Button(23)
esc = Servo(22,min_pulse_width=1e-3,   # 1.0 ms
            max_pulse_width=2e-3,   # 2.0 ms
            frame_width=20e-3,      # 50 Hz
            pin_factory=PiGPIOFactory())

#esc.value = 0.0   # 1.5ms neutral
#esc.value = 0.6   # forward
#esc.value = -0.4  # reverse (if ESC supports it)

def testing():
    esc.value=-0.3
    sleep(2)
    esc.value=0.0
    print("motor is on")

while True:
    if PS.is_pressed:
        testing()
        print("pressing")
        led.on()
        sleep(0.25)
        led.off()
    else:
        print("no pressing")
    



