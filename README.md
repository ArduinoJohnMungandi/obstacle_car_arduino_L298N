# obstacle_car_arduino_L298N

# Robot Car with Ultrasonic Sensor and Servo Motor

This project involves building a robot car that can move forward, backward, stop, and turn based on distance measurements from an ultrasonic sensor. The car uses an L298N motor controller to drive two DC motors and a servo motor to rotate the ultrasonic sensor for obstacle detection.

## Required Libraries
- [NewPing Library](https://bitbucket.org/teckel12/arduino-new-ping/downloads/): Handles the ultrasonic sensor.
- Servo Library: Included with the Arduino IDE.

## Hardware Components
1. Arduino Uno (or compatible board)
2. L298N Motor Controller
3. Two DC Motors
4. Servo Motor
5. Ultrasonic Sensor (HC-SR04)
6. Jumper Wires
7. Breadboard (optional)

## Pin Configuration
- **Motor A (Left Motor)**
  - `enA` - Enable pin for motor speed control (PWM): Pin 6
  - `in1` - Motor direction: Pin 8
  - `in2` - Motor direction: Pin 2

- **Motor B (Right Motor)**
  - `enB` - Enable pin for motor speed control (PWM): Pin 5
  - `in3` - Motor direction: Pin 4
  - `in4` - Motor direction: Pin 3

- **Servo Motor**
  - `servoPin` - Servo control: Pin 9

- **Ultrasonic Sensor**
  - `TRIG_PIN` - Trigger pin: Analog Pin A0
  - `ECHO_PIN` - Echo pin: Analog Pin A1

## Constants
- `MAX_DISTANCE` - Maximum distance for ultrasonic sensor: 200 cm
- `MAX_SPEED` - Maximum speed for DC motors: 80 (out of 255)

## Code Explanation

### Initialization
In the `setup()` function:
- Motor control pins are initialized as outputs.
- Motors are initialized to stopped state.
- Servo motor is attached to the specified pin and positioned at 115 degrees.
- Initial distance readings from the ultrasonic sensor are taken.

### Main Loop
In the `loop()` function:
- The robot checks if an obstacle is within 40 cm.
- If an obstacle is detected, the robot stops, moves backward, and then chooses to turn left or right based on distance measurements from both directions.
- If no obstacle is detected, the robot moves forward.

### Functions

#### `moveStop()`
Stops both motors by setting their speed to 0.

#### `moveForward()`
Sets the motor directions to forward and gradually increases the speed to `MAX_SPEED`.

#### `moveBackward()`
Sets the motor directions to backward and gradually increases the speed to `MAX_SPEED`.

#### `turnRight()`
Turns the robot to the right by running the left motor forward and the right motor backward.

#### `turnLeft()`
Turns the robot to the left by running the left motor backward and the right motor forward.

#### `lookRight()`
Rotates the servo to the right, measures the distance, and returns the distance value.

#### `lookLeft()`
Rotates the servo to the left, measures the distance, and returns the distance value.

#### `readPing()`
Measures the distance using the ultrasonic sensor and returns the distance in cm. If no object is detected, it returns 250 cm.

## Example Code

## Additional Notes
- Ensure that the NewPing library is installed before uploading the code to your Arduino board.
- The servo library is included by default in the Arduino IDE.
- Adjust the delays and motor speeds as needed for your specific robot car setup.
- This project can be expanded with additional sensors or functionality as needed.
