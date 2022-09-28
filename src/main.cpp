#include <Arduino.h>
#include <assert.h>
#include <AFMotor.h>

#define PIN_OPT_SENSOR A0

#define PIN_FRONT_RANGE_ECHO 2
#define PIN_FRONT_RANGE_TRIGGER 3

#define PIN_RIGHT_RANGE_ECHO 4
#define PIN_RIGHT_RANGE_TRIGGER 5

#define PIN_LEFT_RANGE_ECHO 6
#define PIN_LEFT_RANGE_TRIGGER 7

#define FRONT_RANGE 1
#define RIGHT_RANGE 2
#define LEFT_RANGE 3

struct US_RANGE_ECHO_PINS {
  int ECHO_FRONT, ECHO_RIGHT, ECHO_LEFT;
};

struct US_RANGE_TRIGGER_PINS {
  int TRIGGER_FRONT, TRIGGER_RIGHT, TRIGGER_LEFT;
};

US_RANGE_ECHO_PINS echoes = {PIN_FRONT_RANGE_ECHO, PIN_RIGHT_RANGE_ECHO, PIN_LEFT_RANGE_ECHO};
US_RANGE_TRIGGER_PINS triggers = {PIN_FRONT_RANGE_TRIGGER, PIN_RIGHT_RANGE_TRIGGER, PIN_LEFT_RANGE_TRIGGER};

int get_struct_echo(US_RANGE_ECHO_PINS s, int i){
  switch(i) {
    case 0: return s.ECHO_FRONT;
    case 1: return s.ECHO_RIGHT;
    case 2: return s.ECHO_LEFT;
  }
  assert(0);
}

int get_struct_trigger(US_RANGE_TRIGGER_PINS s, int i){
  switch(i) {
    case 0: return s.TRIGGER_FRONT;
    case 1: return s.TRIGGER_RIGHT;
    case 2: return s.TRIGGER_LEFT;
  }
  assert(0);
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

  int sensor_val = analogRead(PIN_OPT_SENSOR);
  return sensor_val;
}

void setup() {
  for (int i=0; i<3; i++){
    pinMode(get_struct_echo(echoes, i), INPUT);
    pinMode(get_struct_trigger(triggers, i), OUTPUT);
  }


  Serial.begin(9600);
  Serial.println("Serial init");
}

void loop() {
  char buf[16];

  int dis1 = get_distance(FRONT_RANGE);
  int dis2 = get_distance(RIGHT_RANGE);
  int dis3 = get_distance(LEFT_RANGE);
  Serial.println(dis1);
  snprintf(buf, sizeof(buf), "%d %d %d", dis1, dis2, dis3);

  Serial.println(buf);
  delay(10);
}