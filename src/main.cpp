#include <Arduino.h>
#include <AFMotor.h>

#define PIN_OPT_SENSOR A0

#define PIN_RANGE_ECHO 2
#define PIN_RANGE_TRIGGER 3

int calc_centrimeters(long duration){
  /**
   * @brief Description of calc_centimeters function.
   * @param duration The length of the pulse (in microseconds) or 0 if no pulse started before the timeout. Data type: unsigned long.
   * @return distance in centimeters.
   */

  return (duration/2) * 0.0343;
}

void clear_trigPIN(){
  digitalWrite(PIN_RANGE_TRIGGER, LOW);
}

int get_distance(){
  /**
   * @brief Get distance from base to the object.
   * @return centimeters to the object.
   */

  clear_trigPIN();

  digitalWrite(PIN_RANGE_TRIGGER, HIGH);
  delay(10);
  digitalWrite(PIN_RANGE_TRIGGER, LOW);

  long duration = pulseIn(PIN_RANGE_ECHO, HIGH);
  int distance = calc_centrimeters(duration);

  return distance;
}

int optical_sensor(){
  int sensor_val = analogRead(PIN_OPT_SENSOR);
  return sensor_val;
}

void setup() {
  pinMode(PIN_RANGE_ECHO, INPUT);
  pinMode(PIN_RANGE_TRIGGER, OUTPUT);

  Serial.begin(9600);
  Serial.println("Serial init");
}

void loop() {
  Serial.println(optical_sensor());
  delay(1000);
}