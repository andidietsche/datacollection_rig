import RPi.GPIO as GPIO
import time

# Pin definitions
DIR_PIN = 20  # Direction pin
STEP_PIN = 21  # Step pin
ENABLE_PIN = 16  # Enable pin (optional)
ALM_PIN = 12  # Alarm pin
LIMIT_PIN = 14 # When at limit


STEPS_PER_REVOLUTION = 200  # Number of steps per full revolution (1.8 degrees per step)


# add This to a class?
#Generic class to add steper motors
class Stepper():
    def __init__(self, step_pin, dir_pin, alm_pin, top_limit_pin, ena_pin=None, bottom_limit_pin=None, step_angle=1.8):
        """
        step_pin: Step pin (Pul+)
        dir_pim : Direction pin (Dir+)
        ena_pin : Enable pin (optional)
        alm_pin : Alarm pin
        top_limit_pin: top limit swtich 
        bottom_limit_pin : bottom limit switch
        step_anlge : motor specific (mostly 1.8 degrees per step)

        """
        #set pins
        self.DIR_PIN = dir_pin
        self.STEP_PIN = step_pin
        self.ENABLE_PIN = ena_pin
        self.ALM_PIN = alm_pin
        self.TOP_LIMIT_PIN = top_limit_pin
        self.BOTTOM_LIMIT_PIN = bottom_limit_pin
    
        self.STEPS_PER_REVOLUTION = 360.0/step_angle  # Number of steps per full revolution (1.8 degrees per step)
        self.manual_control_active = True


    def setup(self):
        # Use the broadcom layout for the GPIO
        GPIO.setmode(GPIO.BCM)
        # Set up the GPIO pins
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT) #TODO not working at the moment
        GPIO.setup(self.ALM_PIN,  GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Assuming ALM pin is active low
        GPIO.setup(self.TOP_LIMIT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.BOTTOM_LIMIT_PIN:
            GPIO.setup(self.BOTTOM_LIMIT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.ENABLE_PIN:
            GPIO.output(self.ENABLE_PIN, GPIO.LOW) # Set the enable pin to Low to enable the motor

    def set_direction(self, clockwise=True):
        if clockwise:
            GPIO.output(self.DIR_PIN, GPIO.HIGH)
        else:
            GPIO.output(self.DIR_PIN, GPIO.LOW)

    def step(self, delay):
        GPIO.output(self.STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(self.STEP_PIN, GPIO.LOW)
        time.sleep(delay)
        
    def stop_motor(self):
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)
        print("Motor stopped.")

    def calibrate(self):
        # Move to the top until reaching the limit switch
        # Up is clockwise for the motor
        self.set_direction(clockwise=True)
        while GPIO.input(self.TOP_LIMIT_PIN) == GPIO.HIGH:
            self.step(0.001)
        # Move back a bit
        self.set_direction(clockwise=False)
        for _ in range(50):
            self.step(0.001)
        
    
        

    def check_alarm(self): 
        return GPIO.input(self.ALM_PIN) == GPIO.LOW
    
    def dist_to_steps(self, dist: float):
        #dist has to be expressed in mm
        return dist/8

    def set_command(self, clockwise=True):
        self.set_direction(clockwise)
        delay = 1.0 / 100 / 2.0 # 100 = speed
        steps = self.dist_to_steps(100) # 100mm



    
