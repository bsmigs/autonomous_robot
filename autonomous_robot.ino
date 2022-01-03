// Define pins:
#define ULTRASONIC_TRIG_PIN 2
#define ULTRASONIC_ECHO_PIN 3
#define L298N_1_ENABLE_PIN 4
#define L298N_1_IN1_PIN 5
#define L298N_1_IN2_PIN 6
#define L298N_1_IN3_PIN 7
#define L298N_1_IN4_PIN 8
#define L298N_2_ENABLE_PIN 9
#define L298N_2_IN1_PIN 10
#define L298N_2_IN2_PIN 11
#define L298N_2_IN3_PIN 12
#define L298N_2_IN4_PIN 13
#define BAUD_RATE 9600
#define MAX_EIGHT_BIT_VALUE 255

// Define constant variables:
const float speed_of_sound_cm_per_us = 0.034;
const float trig_pin_high_time_us = 10.0;
const float generic_delay_time_us = 5.0;
const int n_pts = 10;
const float min_duty_cycle = 30.0; // percentage
const float max_duty_cycle = 100.0; // percentage

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
  pinMode(L298N_1_ENABLE_PIN, OUTPUT);
  pinMode(L298N_1_IN1_PIN, OUTPUT);
  pinMode(L298N_1_IN2_PIN, OUTPUT);
  pinMode(L298N_1_IN3_PIN, OUTPUT);
  pinMode(L298N_1_IN4_PIN, OUTPUT);

  // for the second motor controller (back two motors)
  pinMode(L298N_2_ENABLE_PIN, OUTPUT);
  pinMode(L298N_2_IN1_PIN, OUTPUT);
  pinMode(L298N_2_IN2_PIN, OUTPUT);
  pinMode(L298N_2_IN3_PIN, OUTPUT);
  pinMode(L298N_2_IN4_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(L298N_1_IN1_PIN, LOW);
  digitalWrite(L298N_1_IN2_PIN, LOW);
  digitalWrite(L298N_1_IN3_PIN, LOW);
  digitalWrite(L298N_1_IN4_PIN, LOW);
  digitalWrite(L298N_2_IN1_PIN, LOW);
  digitalWrite(L298N_2_IN2_PIN, LOW);
  digitalWrite(L298N_2_IN3_PIN, LOW);
  digitalWrite(L298N_2_IN4_PIN, LOW);
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
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  
  // Calculate the distance:
  float distance_cm = duration * (0.5 * speed_of_sound_cm_per_us);

  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(50);

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

  return avg_dist_cm;
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
                    enum motor_state function)
{
  int pin1, pin2;
  switch (pos)
  {
    case FRONT_LEFT:
      pin1 = L298N_1_IN1_PIN;
      pin2 = L298N_1_IN2_PIN;
      break;
    case FRONT_RIGHT:
      pin1 = L298N_1_IN3_PIN;
      pin2 = L298N_1_IN4_PIN;
      break;
    case BACK_LEFT:
      pin1 = L298N_2_IN1_PIN;
      pin2 = L298N_2_IN2_PIN;
      break;
    case BACK_RIGHT:
      pin1 = L298N_2_IN3_PIN;
      pin2 = L298N_2_IN4_PIN;
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

void go_forward()
{
  motor_function(FRONT_LEFT, FORWARD);
  motor_function(FRONT_RIGHT, FORWARD);
  motor_function(BACK_LEFT, FORWARD);
  motor_function(BACK_RIGHT, FORWARD);
}

void go_backward()
{
  motor_function(FRONT_LEFT, BACKWARD);
  motor_function(FRONT_RIGHT, BACKWARD);
  motor_function(BACK_LEFT, BACKWARD);
  motor_function(BACK_RIGHT, BACKWARD);
}

void go_forward_right()
{
  motor_function(FRONT_LEFT, FORWARD);
  motor_function(FRONT_RIGHT, BACKWARD);
  motor_function(BACK_LEFT, FORWARD);
  motor_function(BACK_RIGHT, BACKWARD);
}

void go_forward_left()
{
  motor_function(FRONT_LEFT, BACKWARD);
  motor_function(FRONT_RIGHT, FORWARD);
  motor_function(BACK_LEFT, BACKWARD);
  motor_function(BACK_RIGHT, FORWARD);
}

void go_backward_right()
{
  go_forward_left();
}

void go_backward_left()
{
  go_forward_right();
}

void full_stop()
{
  motor_function(FRONT_LEFT, STOP);
  motor_function(FRONT_RIGHT, STOP);
  motor_function(BACK_LEFT, STOP);
  motor_function(BACK_RIGHT, STOP);
}

void setup() 
{
  // setup HC-SR04 ultrasonic sensor pins
  ultrasonic_setup();

  // setup L298N H-bridge motor controller pins
  motor_controller_setup();
  
  //Begin Serial communication at a baudrate of 9600:
  Serial.begin(BAUD_RATE);
}

void loop() 
{
  
  
  
}
