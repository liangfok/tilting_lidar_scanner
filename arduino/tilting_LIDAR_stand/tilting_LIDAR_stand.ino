/*!
 * This is the Arduino program for the tilting scanning LIDAR system.
 *
 * When powered up, it first re-calibrates its starting position by
 * tilting the LIDAR sensor forward/down until the upper limit switch is
 * disabled. It then slowly tilts the stand backwards/up until the upper
 * limit switch is hit. At this point, the tilting stand is at the 0
 * degree position.
 *
 * Once calibrated, the program monitors incoming commands from the serial
 * port. There are two possible commands: (1) init and (2) step. The init
 * command tells this program to re-run the initial calibration routine.
 * The step command tells this program to make the stepper motor take a step.
 * The direction of the step is decided by this program. During this phase,
 * this program sends the current angle of the LIDAR platform over the serial
 * port.
 */

/*===========================================================================*/
enum State
{
  RECALIBRATE, // re-calibrates the position of the tilt sensor using the upper limit switch
  ACTIVE       // normal operation
};

enum RecalibrateState
{
  INIT,
  TILT_FORWARD,
  TILT_BACKWARD
};

State state = RECALIBRATE; // The current state of this program
RecalibrateState recalibrateState = INIT;  // The current state of the recalibration procedure

const int SETUP_DELAY_MS = 1000; // The amount of time to delay after setup.

// Pin definitions for communicating with the stepper motor controller
const int STEP_PIN = 2;
const int DIRECTION_PIN = 3;
const int MS1_PIN = 5;
const int MS2_PIN = 4;
const int EN_PIN = 6;

// Variables for blinking a status LED
const int LED_PIN = 13;
int statusLEDState = LOW;
int currLEDState = LOW;
const long LED_BLINK_PERIOD = 1000;
const long LED_BLINK_INTERVAL_RECALIBRATE = 750;  // blink interval in milliseconds during RECALIBRATE state
const long LED_BLINK_INTERVAL_ACTIVE = 200;       // blink interval in milliseconds during ACTIVE state
unsigned long prevBlinkTime = 0; // The time during the previous blink time

// Variables for the two LEDs
const int LED1_PIN = 7;
const int LED2_PIN = 10;

// Variables for the limit switches
const int UPPER_LIMIT_SWITCH_PIN = 8;
const int LOWER_LIMIT_SWITCH_PIN = 9;
int upperLimitSwitchState = LOW;
int lowerLimitSwitchState = LOW;

// The amount of time to delay after taking a step
const int STEP_DELAY_US = 128000;
//const int STEP_DELAY_US = 250;

const int RECALIBRATE_STEP_DELAY_US = 128000;
const int RECALIBREATE_TILT_PAUSE_TIME = 1000;

// The stepper motor provides 400 steps per revolution and there is a 3X
// reduction from the motor to the stand (20 tooth pulley on motor and
// 60-tooth pulley on the stand). Thus, there are 400 * 3 = 1200 steps per revolution of the stand.
// Since we are taking 1/8th steps, there are 1200 * 8 = 9600 steps per revolution.
const int STEPS_PER_REV = 4800; // 4,800 steps per revolution with 1/8th steps ?? Could this be 1/4 step?

// Input parameters to method stepMotor(...)
const bool FORWARD = true;
const bool BACKWARD = false;

// Define the communication protocol with the control PC.
// The protocol consists of two bytes, a start byte followed by a command byte.
const int COMMAND_START_BYTE = 0x55;  // the start byte of an incoming message
const int COMMAND_NONE = 0;
const int COMMAND_STEP = 1;
const int COMMAND_RECALIBRATE = 2;

int cmd = COMMAND_NONE; // The command from the Control PC.

const int MAX_STEP_POSITION = 1200; // The forward-most step position

int  stepPosition  = 0;       // Where the stepper motor is right now
bool stepDirection = FORWARD; // The step direction

// double angle = 0; // angle of the LIDAR sensor

const byte ANGLE_MSG_START_BYTE = 0x66; // the start byte of an angle message
byte angleMsg[6];  // byte order: [start byte] [4 byte int - stepPosition] [checksum byte], checksum is xor of all previous bytes

/*===========================================================================*/

/*!
 *  Called once at power on. It initializes the serial port and the I/O
 *  pins.
 */
void setup()
{
    Serial.begin(9600);

    // Setup connection to stepper motor driver
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIRECTION_PIN, LOW);
    digitalWrite(EN_PIN, LOW);

    // Place the stepper motor driver into 1/8th steps
    digitalWrite(DIRECTION_PIN, HIGH);
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);

    // Setup status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Setup the diagnostic LEDs
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);

    // Setup the limit switch pins
    pinMode(UPPER_LIMIT_SWITCH_PIN, INPUT);
    pinMode(LOWER_LIMIT_SWITCH_PIN, INPUT);

    //delay(SETUP_DELAY_MS);  // Is this necessary?

    angleMsg[0] = ANGLE_MSG_START_BYTE;
    recalibrateState = INIT;
}

/*!
 *  Periodically called.
 */
void loop()
{
    blinkStatusLED();

    checkLimitSwitches();

    if (state == RECALIBRATE)
        doRecalibrate();
    else
        doActiveTask();
}

/*!
 *  Blinks the status LED. The LED's blink pattern is based on the current state.
 *  
 *  @param state The current state
 */
void blinkStatusLED()
{
    unsigned long currBlinkTime = millis();

    // Check if it's time to start a new blink interval. If it is, record the begin time of
    // this new interval.
    if (currBlinkTime - prevBlinkTime >= LED_BLINK_PERIOD)
        prevBlinkTime = currBlinkTime;

    // Determine the blink period based on the current state
    unsigned long blinkPeriod = LED_BLINK_INTERVAL_RECALIBRATE;
    if (state == ACTIVE)
        blinkPeriod = LED_BLINK_INTERVAL_ACTIVE;

    // Determine whether the status LED should be on or off
    statusLEDState = LOW;
    if (currBlinkTime - prevBlinkTime <= blinkPeriod)
        statusLEDState = HIGH;

    // Update the LED's state if necessary.
    if (currLEDState != statusLEDState)
    {
        digitalWrite(LED_PIN, statusLEDState);
        currLEDState = statusLEDState;
    }
}

/*!
 *  Read the current state of the lower and upper limit switches.
 */
void checkLimitSwitches()
{
    upperLimitSwitchState = digitalRead(UPPER_LIMIT_SWITCH_PIN);
    lowerLimitSwitchState = digitalRead(LOWER_LIMIT_SWITCH_PIN);

    // For debugging purposes...
    digitalWrite(LED1_PIN, upperLimitSwitchState);
    digitalWrite(LED2_PIN, lowerLimitSwitchState);
}

/*!
 *  Tilts the sensor forward until the upper limit switch is no longer pressed.
 *  Then slowly tilts the sensor up until the upper limit switch is pressed.
 *  Saves this state as the zero angle and changes the program state to be ACTIVE.
 */
void doRecalibrate()
{ 
    // During state INIT, determine the tilt direction based on whether the
    // upper limit switch is pressed. If pressed tilt forward, otherwise 
    // tilt backward.
    if (recalibrateState == INIT)
    {
        if (upperLimitSwitchState == HIGH)
            recalibrateState = TILT_FORWARD;
        else  
            recalibrateState = TILT_BACKWARD;
    }

    // Tilt forward until the upper limit switch is no longer pressed.
    // Once the upper limit switch is not pressed, switch to the tilt backward state.
    else if (recalibrateState == TILT_FORWARD)
    {
        if (upperLimitSwitchState == HIGH)
        {
            stepMotor(FORWARD);
            delayMicroseconds(RECALIBRATE_STEP_DELAY_US);
        }
        else
        {
            recalibrateState = TILT_BACKWARD;
            delay(RECALIBREATE_TILT_PAUSE_TIME);
        }
    }

    else if (recalibrateState == TILT_BACKWARD)
    {
        if (upperLimitSwitchState == HIGH)
        {
            recalibrateState = INIT;                 // reset recalibrate procedure to init state
            stepPosition = 0;                        // reset step position to be at zero
            stepDirection = FORWARD;                 // reset step direction to be forward
            state = ACTIVE;                          // go into ACTIVE state
            delay(RECALIBREATE_TILT_PAUSE_TIME);
        }
        else
        {
            stepMotor(BACKWARD);
            delayMicroseconds(RECALIBRATE_STEP_DELAY_US);
        }
    }
}

void doActiveTask()
{
    readCommand();

    if (cmd == COMMAND_RECALIBRATE)
        state = RECALIBRATE;
    else if (cmd == COMMAND_STEP)
    {
        takeStep();
        sendAngle();
    }
}

/*!
 *  Makes the motor take a step. The direction is specified as an input parameter.
 *  
 *  @param goForward Whether to step forward.
 */
void stepMotor(bool goForward)
{
    // Set the direction pin
    if (goForward)
        digitalWrite(DIRECTION_PIN, HIGH);
    else
        digitalWrite(DIRECTION_PIN, LOW);

    // Transition the step pin from low to high. This causes the
    // motor driver to take a step.
    digitalWrite(STEP_PIN, HIGH); // Generates the "Rising Edge" that tells the easydriver when to step.
    digitalWrite(STEP_PIN, LOW);  // This LOW to HIGH change is what creates the
}

/*!
 *  Reads a command from the serial port. Sets global variable "cmd" to be the command.
 */
void readCommand() 
{
    if (Serial.available() >= 2)
    {
        if (Serial.read() == COMMAND_START_BYTE)
        {
            cmd = Serial.read();

            // If the command is invalid, set the command to be NONE
            if (cmd != COMMAND_STEP && cmd != COMMAND_RECALIBRATE)
                cmd = COMMAND_NONE;
        } 
        else
            cmd = COMMAND_NONE;
    } else
        cmd = COMMAND_NONE;
}


void takeStep() 
{
    if (stepDirection == FORWARD)
    {
        if (stepPosition < MAX_STEP_POSITION)
        {
            stepMotor(FORWARD);
            delayMicroseconds(STEP_DELAY_US);

            if (++stepPosition == MAX_STEP_POSITION) 
              stepDirection = BACKWARD;
        }
        else
            stepDirection = BACKWARD;
    }
    else 
    {
        if (stepPosition > 0)
        {
            stepMotor(BACKWARD);
            delayMicroseconds(STEP_DELAY_US);
           
            if (--stepPosition == 0)
                stepDirection = FORWARD;
        }
        else
            stepDirection = FORWARD;
    }
}

void sendAngle()
{
//    angle = ((double) stepPosition / (double) STEPS_PER_REV) * 360;

    angleMsg[1] = (byte)((stepPosition >> 24) & 0xFF);
    angleMsg[2] = (byte)((stepPosition >> 16) & 0xFF);
    angleMsg[3] = (byte)((stepPosition >> 8) & 0xFF);
    angleMsg[4] = (byte)(stepPosition & 0xFF);

    angleMsg[5] = angleMsg[0] ^ angleMsg[1] ^ angleMsg[2] ^ angleMsg[3] ^ angleMsg[4];
    
    Serial.write(angleMsg, sizeof(angleMsg));
    Serial.flush();
}

