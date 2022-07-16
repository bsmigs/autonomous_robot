#include <Servo.h>

// Define pins:
#define ULTRASONIC_TRIG_PIN 46
#define ULTRASONIC_ECHO_PIN 48
#define BACK_ENABLE_PIN_A 8
#define BACK_ENABLE_PIN_B 9
#define BACK_IN1_PIN 24
#define BACK_IN2_PIN 26
#define BACK_IN3_PIN 30
#define BACK_IN4_PIN 32
#define FRONT_ENABLE_PIN_A 2
#define FRONT_ENABLE_PIN_B 3
#define FRONT_IN1_PIN 36
#define FRONT_IN2_PIN 38
#define FRONT_IN3_PIN 40
#define FRONT_IN4_PIN 42
#define BAUD_RATE 9600
#define MAX_EIGHT_BIT_VALUE 255
#define DEBUG 0
#define RANGE_FINDER_SERVO_PIN 10

/*
   Front L298N motor controller: Out1 = green, Out2 = red, Out3 = green, Out4 = red
   Back L298N motor controller: Out1 = green, Out2 = red, Out3 = green, Out4 = red
*/

// Define constant variables:
const float speed_of_sound_cm_per_us = 0.034;
const float trig_pin_high_time_us = 10.0;
const float generic_delay_time_us = 5.0;
const int n_pts = 50;
const float min_duty_cycle = 16.0; // percentage
const float max_duty_cycle = 100.0; // percentage
const float nominal_duty_cycle = 35.0; // percentage
const float stop_dist_cm = 35.0;
const float ewma_alpha = 0.8; // alpha constant for exponentially weighted moving average alg
const float max_dist_cm = 200;
const float max_duration = 2 * max_dist_cm / speed_of_sound_cm_per_us;
const int turn_delay = 250;

float ewma_dist_cm = 0.0; // exponentially weighted moving average global var

Servo range_finder_servo;

enum motor_state
{
  FORWARD,
  BACKWARD,
  STOP
};

enum motor_position
{
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};

enum rfs_servo_angles
{
  RIGHT = 0,
  STRAIGHT = 90,
  LEFT = 180
};

// Define methods
void ultrasonic_setup()
{
  // Define inputs and outputs:
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

void motor_controller_setup()
{
  // for the first motor controller (front two motors)
  pinMode(FRONT_ENABLE_PIN_A, OUTPUT);
  pinMode(FRONT_ENABLE_PIN_B, OUTPUT);
  pinMode(FRONT_IN1_PIN, OUTPUT);
  pinMode(FRONT_IN2_PIN, OUTPUT);
  pinMode(FRONT_IN3_PIN, OUTPUT);
  pinMode(FRONT_IN4_PIN, OUTPUT);

  // for the second motor controller (back two motors)
  pinMode(BACK_ENABLE_PIN_A, OUTPUT);
  pinMode(BACK_ENABLE_PIN_B, OUTPUT);
  pinMode(BACK_IN1_PIN, OUTPUT);
  pinMode(BACK_IN2_PIN, OUTPUT);
  pinMode(BACK_IN3_PIN, OUTPUT);
  pinMode(BACK_IN4_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(FRONT_IN1_PIN, LOW);
  digitalWrite(FRONT_IN2_PIN, LOW);
  digitalWrite(FRONT_IN3_PIN, LOW);
  digitalWrite(FRONT_IN4_PIN, LOW);
  digitalWrite(BACK_IN1_PIN, LOW);
  digitalWrite(BACK_IN2_PIN, LOW);
  digitalWrite(BACK_IN3_PIN, LOW);
  digitalWrite(BACK_IN4_PIN, LOW);

}

float get_ultrasonic_distance_cm()
{
  // Clear the ULTRASONIC_TRIG_PIN by setting it LOW:
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(generic_delay_time_us);

  // Trigger the sensor by setting the ULTRASONIC_TRIG_PIN high for 10 microseconds:
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(trig_pin_high_time_us);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Read the ULTRASONIC_ECHO_PIN, pulseIn() returns the duration (length of the pulse) in microseconds:
  long duration_us = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

  // Calculate the distance:
  float distance_cm = duration_us * (0.5 * speed_of_sound_cm_per_us);

  return distance_cm;
}

float get_averaged_ultrasonic_distance_cm()
{
  float avg_dist_cm = 0.0;
  for (int ii = 0; ii < n_pts; ii++)
  {
    avg_dist_cm += get_ultrasonic_distance_cm();
  }
  avg_dist_cm /= n_pts;

  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
#ifdef DEBUG
  Serial.print("Average distance = ");
  Serial.print(avg_dist_cm);
  Serial.println(" cm");
  delay(50);
#endif

  return avg_dist_cm;
}

float get_ewma_ultrasonic_distance_cm()
{
  float new_ewma_data = get_ultrasonic_distance_cm();
  ewma_dist_cm = ewma_alpha * new_ewma_data + (1 - ewma_alpha) * ewma_dist_cm;

  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
#ifdef DEBUG
  Serial.print("Average distance = ");
  Serial.print(ewma_dist_cm);
  Serial.println(" cm");
  delay(50);
#endif

  return ewma_dist_cm;
}

int convert_duty_cycle(float &duty_cycle)
{
  // saw empirically that is duty cycle is less than 30% it just buzzed and didn't spin the motors
  // Maybe this is voltage dependent? I used a 9V battery. Will have to experiment with this
  if (duty_cycle < min_duty_cycle)
  {
    duty_cycle = min_duty_cycle;
  }

  // if the value is more than 100%, cap it at 100%
  if (duty_cycle > max_duty_cycle)
  {
    duty_cycle = max_duty_cycle;
  }

  // convert duty cycle to a number between 0 and 255 so it's 8-bits
  int eight_bit_value = floor(MAX_EIGHT_BIT_VALUE * (duty_cycle / max_duty_cycle));

  return eight_bit_value;
}

void motor_function(enum motor_position pos,
                    enum motor_state function,
                    float duty_cycle)
{

  int eight_bit_speed = convert_duty_cycle(duty_cycle);

  int pin1, pin2;
  switch (pos)
  {
    case FRONT_LEFT:
      pin1 = FRONT_IN3_PIN;
      pin2 = FRONT_IN4_PIN;
      analogWrite(FRONT_ENABLE_PIN_B, eight_bit_speed);
      break;
    case FRONT_RIGHT:
      pin1 = FRONT_IN1_PIN;
      pin2 = FRONT_IN2_PIN;
      analogWrite(FRONT_ENABLE_PIN_A, eight_bit_speed);
      break;
    case BACK_LEFT:
      pin1 = BACK_IN3_PIN;
      pin2 = BACK_IN4_PIN;
      analogWrite(BACK_ENABLE_PIN_B, eight_bit_speed);
      break;
    case BACK_RIGHT:
      pin1 = BACK_IN1_PIN;
      pin2 = BACK_IN2_PIN;
      analogWrite(BACK_ENABLE_PIN_A, eight_bit_speed);
      break;
  }

  switch (function)
  {
    case FORWARD: // forward
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      break;
    case BACKWARD: // backward
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      break;
    case STOP: // stop
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      break;
    default: // stop
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      break;
  }
}

void go_forward(float duty_cycle)
{
  motor_function(FRONT_LEFT, FORWARD, duty_cycle);
  motor_function(FRONT_RIGHT, FORWARD, duty_cycle);
  motor_function(BACK_LEFT, FORWARD, duty_cycle);
  motor_function(BACK_RIGHT, FORWARD, duty_cycle);
}

void go_backward(float duty_cycle)
{
  motor_function(FRONT_LEFT, BACKWARD, duty_cycle);
  motor_function(FRONT_RIGHT, BACKWARD, duty_cycle);
  motor_function(BACK_LEFT, BACKWARD, duty_cycle);
  motor_function(BACK_RIGHT, BACKWARD, duty_cycle);
}

void spin_right(float duty_cycle)
{
  motor_function(FRONT_LEFT, FORWARD, duty_cycle);
  motor_function(FRONT_RIGHT, BACKWARD, duty_cycle + 20);
  motor_function(BACK_LEFT, FORWARD, duty_cycle);
  motor_function(BACK_RIGHT, BACKWARD, duty_cycle + 20);
}

void spin_left(float duty_cycle)
{
  motor_function(FRONT_LEFT, FORWARD, duty_cycle + 20);
  motor_function(FRONT_RIGHT, FORWARD, duty_cycle);
  motor_function(BACK_LEFT, FORWARD, duty_cycle + 20);
  motor_function(BACK_RIGHT, FORWARD, duty_cycle);
}

void full_stop()
{
  motor_function(FRONT_LEFT, STOP, 0);
  motor_function(FRONT_RIGHT, STOP, 0);
  motor_function(BACK_LEFT, STOP, 0);
  motor_function(BACK_RIGHT, STOP, 0);
}


/*
    UNIT TESTS, SETUP, AND LOOP METHOD
*/

void test_1()
{
  // Test 1: Drive "front" 2 motors
  motor_function(FRONT_LEFT, FORWARD, 60);
  motor_function(FRONT_RIGHT, FORWARD, 60);
  delay(3000); // delay 3 sec
  motor_function(FRONT_LEFT, STOP, 0);
  motor_function(FRONT_RIGHT, STOP, 0);
  delay(3000);
}

void test_2()
{
  // Test 2: Drive "back" 2 motors
  motor_function(BACK_LEFT, FORWARD, 60);
  motor_function(BACK_RIGHT, FORWARD, 60);
  delay(3000); // delay 3 sec
  motor_function(BACK_LEFT, STOP, 0);
  motor_function(BACK_RIGHT, STOP, 0);
  delay(3000);
}

void test_3()
{
  // Test 3: Drive all motors forward
  go_forward(nominal_duty_cycle);
  delay(3000); // delay 3 sec
  full_stop();
  delay(3000);
}

void test_3a()
{
  // Test 3: spin left
  spin_left(50);
  delay(3000); // delay 3 sec
  full_stop();
  delay(3000);
}

void test_3b()
{
  // Test 3: turn right
  spin_right(50);
  delay(2000); // delay 2 sec
  full_stop();
  delay(2000);
}

void test_4()
{
  // Test 4: Drive all motors backward
  go_backward(nominal_duty_cycle);
  delay(6000); // delay 3 sec
  full_stop();
  delay(2000);
}

void test_5()
{
  // Test 5: Drive forward and then stop when it's in range of a threshold
  //float dist_to_ether = get_averaged_ultrasonic_distance_cm();
  float dist_to_ether = get_ewma_ultrasonic_distance_cm();
  if (dist_to_ether > stop_dist_cm)
  {
    go_forward(nominal_duty_cycle);
  }
  else
  {
    full_stop();
  }

  delay(100);
}

void test_6()
{
  range_finder_servo.write(RIGHT);
  delay(turn_delay);
  range_finder_servo.write(STRAIGHT);
  delay(turn_delay);
  range_finder_servo.write(LEFT);
  delay(turn_delay);
  range_finder_servo.write(STRAIGHT);
  delay(turn_delay);
}



void follow_best_path()
{
  // start off looking forward
  range_finder_servo.write(STRAIGHT);
  delay(5);

  // get the distance ahead
  float curr_range_cm = get_averaged_ultrasonic_distance_cm();
  //float curr_range_cm = get_ewma_ultrasonic_distance_cm();
  if (curr_range_cm >= stop_dist_cm)
  {
    go_forward(nominal_duty_cycle);
  }

  //Keep checking the object distance until it is within the minimum stopping distance
  while (curr_range_cm >= stop_dist_cm)
  {
    curr_range_cm = get_averaged_ultrasonic_distance_cm();
    //curr_range_cm = get_ewma_ultrasonic_distance_cm();
    //Serial.print("range (cm) = ");
    //Serial.println(curr_range_cm);
    delay(2);
  }

  // if we get within stopping distance, stop
  full_stop();
  delay(250);
  //Serial.println("Stopped motors");


  // Assuming the ranger finder servo is connected, want
  // to scan a range of directions to determine where to move.
  // It will check left, forward, and right in order to make
  // the decision

  range_finder_servo.write(RIGHT);
  //float right_range_cm = get_ewma_ultrasonic_distance_cm();
  float right_range_cm = get_averaged_ultrasonic_distance_cm();
  delay(500);
  range_finder_servo.write(LEFT);
  //float left_range_cm = get_ewma_ultrasonic_distance_cm();
  float left_range_cm = get_averaged_ultrasonic_distance_cm();
  delay(500);

/*
  Serial.print("Right range = ");
  Serial.print(right_range_cm);
  Serial.print(", left range = ");
  Serial.println(left_range_cm);
*/


  if (right_range_cm >= max_dist_cm && left_range_cm >= max_dist_cm)
  {
    // if both directions are valid, go right. In the future make this random
    spin_right(nominal_duty_cycle);
    delay(100);
  }
  else if (right_range_cm <= stop_dist_cm && left_range_cm <= stop_dist_cm)
  {
    // both directions are invalid, backup and then turn around
    go_backward(nominal_duty_cycle);
    delay(250);
    spin_right(nominal_duty_cycle);
    delay(200);
  }
  else if (right_range_cm >= left_range_cm)
  {
    // if more room to the right, go right
    spin_right(nominal_duty_cycle);
    delay(100);
  }
  else if (right_range_cm < left_range_cm)
  {
    // if more room to the left, go left
    spin_left(nominal_duty_cycle);
    delay(100);
  }

  
}

void setup()
{
  // setup HC-SR04 ultrasonic sensor pins
  ultrasonic_setup();

  // setup L298N H-bridge motor controller pins
  motor_controller_setup();

  // range finder servo setup
  range_finder_servo.attach(RANGE_FINDER_SERVO_PIN);

  //Begin Serial communication at a baudrate of 9600:
  Serial.begin(BAUD_RATE);

#ifdef DEBUG
  Serial.print("FRONT_LEFT = ");
  Serial.println(FRONT_LEFT);
  Serial.print("FORWARD = ");
  Serial.println(FORWARD);
#endif
}

void loop()
{

  //full_stop();

  // Test 1: Drive "front" 2 motors
  //test_1();

  // Test 2: Drive "back" 2 motors
  //test_2();

  // Test 3: Drive all motors forward
  //test_3();

  //test_3b();

  // Test 4: Drive all motors backward
  //test_4();

  // Test 5: Drive forward and then stop when it's in range of a threshold
  //test_5();

  // Test 6: Include the range finder servo and see if we can determine which
  // directions to move
  //test_6();

  //follow_best_path();
}
