import RPi.GPIO as GPIO
import time
import keyboard
import curses



"""
This is just a test file for the linear module
We are using a TR10x4P2 Trapezgewindespinder RPTS rechts
10 = diameter
4P = 4mm Steigung
2  = 2 threaded

rotatary movmement to lineare movement calculation
#revolutions = Distant[mm]/(p*#threads)
x = 40mm/(4*2)

"""
# Pin definitions
DIR_PIN = 20  # Direction pin
STEP_PIN = 21  # Step pin
ENABLE_PIN = 16  # Enable pin (optional)
ALM_PIN = 12  # Alarm pin
LIMIT_PIN = 14 # When at limit

# Stepper motor specifications
STEPS_PER_REVOLUTION = 200  # Number of steps per full revolution (1.8 degrees per step)
manual_control_active= True
# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.setup(ALM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Assuming ALM pin is active low
GPIO.setup(LIMIT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set the enable pin to low (enable the driver)
GPIO.output(ENABLE_PIN, GPIO.LOW)

def set_direction(clockwise=True):
    if clockwise:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)

def step(delay):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(delay)

def check_alarm(): #not Working at the moment FIX
    return GPIO.input(ALM_PIN) == GPIO.LOW

def calc_angle_from_dist(dist: float):
    angle = dist/8

def rotate_angle(angle, speed, clockwise=True):
    # Calculate the number of steps needed for the given angle
    steps = int((angle / 360.0) * STEPS_PER_REVOLUTION)
    set_direction(clockwise)
    delay = 1.0 / speed / 2.0  # Speed is in steps per second, delay is in seconds
    for _ in range(steps):
        # if check_alarm():
        #     print("Alarm detected! Stopping motor.")
        #     return
        step(delay)
def stop_motor():
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    print("Motor stopped.")

# def manual_control(speed:float):
#     while True:
#         if keyboard.is_pressed("up"):
#             set_direction(clockwise=True)
#             step(1.0/speed/2.0)
#         elif keyboard.is_pressed('down'):
#             set_direction(clockwise=False)
#             step(1.0 / speed / 2.0)
#         elif keyboard.is_pressed('esc'):
#             print("Exiting manual control.")
#             break 
def test_limit():
    switch_state = GPIO.input(LIMIT_PIN)
    if switch_state == GPIO.HIGH:
        print("not at limit")
    elif switch_state == GPIO.LOW:
        print("at limit")
    
def test_speed(desired_RPM,clockwise):
    # Calculate delay between steps based on desired RPM
    delay = 60.0 / (desired_RPM * STEPS_PER_REVOLUTION)
    print("Delay between steps: {:.4f} seconds".format(delay))
    #set direction
    set_direction(clockwise)
    # Move for 400 steps
    for _ in range(400):
        step(delay/2)
    
def test_calibration():
    # Move to the top until reaching the limit switch
    # Up is clockwise for the motor
    set_direction(clockwise=True)
    while GPIO.input(LIMIT_PIN) == GPIO.HIGH:
        step(0.001)
    # Move back a bit
    set_direction(clockwise=False)
    for _ in range(50):
        step(0.001)
    stop_motor()
    
    
    
    
def manual_control(screen, speed):
    steps_per_press = 2000

    screen.nodelay(True)
    delay = 1.0 / speed / 2.0
    while True:
        char = screen.getch()
        if char == curses.KEY_UP:
            set_direction(clockwise=True)
            for _ in range(steps_per_press):
                step(delay)
        elif char == curses.KEY_DOWN:
            set_direction(clockwise=False)
            for _ in range(steps_per_press):
                step(delay)
        elif char == 27:  # ESC key
            print("Exiting manual control.")
            break
        time.sleep(0.01)  # Small delay to prevent CPU overuse
try:
    time.sleep(2)
    if check_alarm():
        print("Initial Alarm detected! Check connections and initial state.")
    else:
        print("No initial alarm detected.")
    
    while True:
        # speed = float(input("Enter speed (steps per second, e.g., 100): "))
        # if manual_control_active:
        #     curses.wrapper(manual_control, speed)
        #     manual_control(speed)
        # else:
        #     angle = float(input("Enter angle of rotation (degrees): "))
        #     direction = input("Enter direction (cw for clockwise, ccw for counterclockwise): ").strip().lower()
        #     # speed = float(input("Enter speed (steps per second, e.g., 100): "))
        #     rotate_angle(angle, speed, clockwise=(direction == 'cw'))
        # test_speed(800, clockwise=True)
        test_calibration()
        # test_limit()
except KeyboardInterrupt:
    pass
finally:
    # Clean up GPIO settings
    GPIO.cleanup()
