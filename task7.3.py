import RPi.GPIO as GPIO
import time

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering
LED = 17
GPIO.setup(LED, GPIO.OUT)

# Set up PWM for the LED
pwm = GPIO.PWM(LED, 100)
pwm.start(0)

# Function to measure distance using the ultrasonic sensor
def UltrasonicSensor():
    TRIG_PIN = 23  # Define TRIG pin
    ECHO_PIN = 24  # Define ECHO pin

    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # Ensure TRIG is initially set to False
    GPIO.output(TRIG_PIN, False)

    # Give some time for the sensor to settle
    time.sleep(2)

    # Send a 10µs pulse to trigger the sensor
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Wait for the ECHO pin to go high (start of pulse)
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    # Wait for the ECHO pin to go low (end of pulse)
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    # Calculate the pulse duration
    pulse_duration = pulse_end - pulse_start

    # Speed of sound is approximately 343 meters per second
    # Distance = (Time x Speed of Sound) / 2
    distance = (pulse_duration * 34300) / 2

    # Ignore readings that are too small (likely noise)
    if distance < 2:
        return None

    distance = round(distance, 2)
    print("Distance:", distance, "cm")
    return distance

try:
    while True:
        distance = UltrasonicSensor()
        if distance is not None:
            # Adjust LED intensity based on distance with increased scaling factor
            scaling_factor = 10  # Adjust this value for more noticeable intensity changes
            duty_cycle = 100 - int(distance * scaling_factor)
            duty_cycle = max(0, min(100, duty_cycle))  # Ensure duty_cycle is within 0-100%
            pwm.ChangeDutyCycle(duty_cycle)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
