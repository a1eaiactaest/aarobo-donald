#include <Arduino.h>

// front-right
#define PIN_OPT_SENSOR_FR A0
// front-left
#define PIN_OPT_SENSOR_FL A1
// rear-right
#define PIN_OPT_SENSOR_RR A2
// rear-left
#define PIN_OPT_SENSOR_RL A3

#define PIN_FRONT_RANGE_ECHO 2
#define PIN_FRONT_RANGE_TRIGGER 3

#define PIN_RIGHT_RANGE_ECHO 4
#define PIN_RIGHT_RANGE_TRIGGER 5

#define PIN_LEFT_RANGE_ECHO 6
#define PIN_LEFT_RANGE_TRIGGER 7

#define FRONT_RANGE 1
#define RIGHT_RANGE 2
#define LEFT_RANGE 3

// Motors with 2x L298N motor drivers
// left
#define PIN_MOTOR1P1 8
#define PIN_MOTOR1P2 9

// right
#define PIN_MOTOR2P1 10
#define PIN_MOTOR2P2 11

// center
#define PIN_MOTOR3P1 12
#define PIN_MOTOR3P2 13

struct MOTOR {
  int pin1, pin2;
};

struct MOTORS {
  MOTOR motor1, motor2, motor3;
};

struct US_RANGE_ECHO_PINS {
  int ECHO_FRONT, ECHO_RIGHT, ECHO_LEFT;
};

struct US_RANGE_TRIGGER_PINS {
  int TRIGGER_FRONT, TRIGGER_RIGHT, TRIGGER_LEFT;
};


MOTOR left_motor = {PIN_MOTOR1P1, PIN_MOTOR1P2};
MOTOR right_motor = {PIN_MOTOR2P1, PIN_MOTOR2P2};
MOTOR center_motor = {PIN_MOTOR3P1, PIN_MOTOR3P2};

MOTORS motors = {left_motor, right_motor, center_motor};
US_RANGE_ECHO_PINS echoes = {PIN_FRONT_RANGE_ECHO, PIN_RIGHT_RANGE_ECHO, PIN_LEFT_RANGE_ECHO};
US_RANGE_TRIGGER_PINS triggers = {PIN_FRONT_RANGE_TRIGGER, PIN_RIGHT_RANGE_TRIGGER, PIN_LEFT_RANGE_TRIGGER};


int get_struct_echo(US_RANGE_ECHO_PINS s, int i){
  switch(i) {
    case 0: return s.ECHO_FRONT;
    case 1: return s.ECHO_RIGHT;
    case 2: return s.ECHO_LEFT;
    default: return -1;
  }
}

int get_struct_trigger(US_RANGE_TRIGGER_PINS s, int i){
  switch(i) {
    case 0: return s.TRIGGER_FRONT;
    case 1: return s.TRIGGER_RIGHT;
    case 2: return s.TRIGGER_LEFT;
    default: return -1;
  }
}

MOTOR get_struct_motors(MOTORS s, int i){
  switch(i) {
    case 0: return s.motor1;
    case 1: return s.motor2;
    case 2: return s.motor3;
  }
}

int get_struct_motor(MOTOR s, int i){
  switch(i) {
    case 0: return s.pin1;
    case 1: return s.pin2;
    default: return -1;
  }
}

int calc_centrimeters(long duration){
  /**
   * @brief Description of calc_centimeters function.
   * @param duration The length of the pulse (in microseconds) or 0 if no pulse started before the timeout. Data type: unsigned long.
   * @return distance in centimeters.
   */

  return (duration/2) * 0.0343;
}

void clear_trigPIN(int pin){
  digitalWrite(pin, LOW);
}

int get_distance(int range_sensor_number){
  /**
   * @brief Get distance from base to the object.
   * @param range_sensor_number Ultrasonic ranging sensor number, can be set to
   *                            FRONT_RANGE, RIGHT_RANGE, LEFT_RANGE.
   * @return centimeters to the object.
   */

  int echo_pin;
  int trig_pin;

  switch (range_sensor_number){
    case FRONT_RANGE:
      echo_pin = PIN_FRONT_RANGE_ECHO;
      trig_pin = PIN_FRONT_RANGE_TRIGGER;
      break;

    case RIGHT_RANGE:
      echo_pin = PIN_RIGHT_RANGE_ECHO;
      trig_pin = PIN_RIGHT_RANGE_TRIGGER;
      break;

    case LEFT_RANGE:
      echo_pin = PIN_LEFT_RANGE_ECHO;
      trig_pin = PIN_LEFT_RANGE_TRIGGER;
      break;

    default:
      Serial.println("LOG: ERROR IN get_distance_function, var:range_sensor_number didn't match.");
      break;
  }

  clear_trigPIN(trig_pin);

  digitalWrite(trig_pin, HIGH);
  delay(10);
  digitalWrite(trig_pin, LOW);

  long duration = pulseIn(echo_pin, HIGH);
  int distance = calc_centrimeters(duration);

  return distance;
}

int read_ROS(int analog_pin){
  /**
   * @brief Read analog input from Reflective Optical Sensor
   *        Reference: https://www.vishay.com/docs/83751/cny70.pdf
   * @param analog_pin Arduino analog pin e.g.: A0, A1, AX...
   */

  int sensor_val = analogRead(analog_pin);
  return sensor_val;
}

String read_US(){
  char buf[40];

  int front_cm = get_distance(FRONT_RANGE);
  int right_cm = get_distance(RIGHT_RANGE);
  int left_cm = get_distance(LEFT_RANGE);

  snprintf(buf, sizeof(buf), "FRONT: %d, RIGHT: %d, LEFT: %d", front_cm, right_cm, left_cm);
  return buf;
}

// TODO
bool on_the_edge(){
  /**
   * @brief Return true if device is on the edge of arena. (detects white line)
   * 
   */
  return true;
}

void motor_forward(MOTOR motor){
  digitalWrite(motor.pin1, HIGH);
  digitalWrite(motor.pin2, LOW);
}

void motor_back(MOTOR motor){
  digitalWrite(motor.pin1, LOW);
  digitalWrite(motor.pin2, HIGH);
}

void motors_run(bool reverse){
  for (int i=0; i<3; i++){
    MOTOR cmotor = get_struct_motors(motors, i);
    if (reverse){
      motor_back(cmotor);
    } else {
      motor_forward(cmotor);
    }
  }
}

void motors_stop(){
  /**
   * @brief Turn off all motors in motors struct.
   */
  for (int i=0; i<6; i++){
    MOTOR cmotor = get_struct_motors(motors, i);
    digitalWrite(cmotor.pin1, LOW);
    digitalWrite(cmotor.pin2, LOW);
  }
}

void turn_left(){
  /**
   * @brief Left, and right motors rotating in opposite directions.
   *        Center motor should be neutral.
   */

  motors_stop();

  // left motor
  digitalWrite(PIN_MOTOR1P1, LOW);
  digitalWrite(PIN_MOTOR1P2, HIGH);

  // right motor
  digitalWrite(PIN_MOTOR2P1, HIGH);
  digitalWrite(PIN_MOTOR2P2, LOW);
}

void turn_right(){
  /**
   * @brief Left, and right motors rotating in opposite directions.
   *        Center motor should be neutral.
   */

  motors_stop();

  // left motor
  digitalWrite(PIN_MOTOR1P1, HIGH);
  digitalWrite(PIN_MOTOR1P2, LOW);

  // right motor
  digitalWrite(PIN_MOTOR2P1, LOW);
  digitalWrite(PIN_MOTOR2P2, HIGH);

}

void debug_log(){
  Serial.println(read_ROS(PIN_OPT_SENSOR_FR));
  Serial.println(read_ROS(PIN_OPT_SENSOR_FL));
  Serial.println(read_ROS(PIN_OPT_SENSOR_RR));
  Serial.println(read_ROS(PIN_OPT_SENSOR_RL));
  Serial.println(read_US());
}

void setup() {
  // setup ultrasonics
  for (int i=0; i<3; i++){
    pinMode(get_struct_echo(echoes, i), INPUT);
    pinMode(get_struct_trigger(triggers, i), OUTPUT);
  }

  // setup motors
  for (int i=0; i<6; i++){
    MOTOR cmotor = get_struct_motors(motors, i);

    // set initial state
    pinMode(cmotor.pin1, OUTPUT);
    pinMode(cmotor.pin2, OUTPUT);
    digitalWrite(cmotor.pin1, LOW);
    digitalWrite(cmotor.pin2, LOW);
  }

  Serial.begin(9600);
  Serial.println("Serial init");
}

void loop() {
  //debug_log();
  delay(10000);
  motors_run(false); //forward
  delay(10000);
  motors_run(true); // back
  delay(10000);
  motors_stop();
  delay(10000);
}

