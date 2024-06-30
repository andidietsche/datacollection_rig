import RPi.GPIO as GPIO
import time

# Pin definition (physical pin number)
ALM_PIN = 32  # Physical pin 32, corresponds to GPIO 12

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ALM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Assuming internal pull-up resistor

def check_alarm():
    return GPIO.input(ALM_PIN) == GPIO.LOW

try:
    print("Monitoring ALM pin. Trigger an alarm condition to test.")
    while True:
        if check_alarm():
            print("Alarm detected! ALM pin is LOW.")
        else:
            print("No alarm. ALM pin is HIGH.")
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    # Clean up GPIO settings
    GPIO.cleanup()
