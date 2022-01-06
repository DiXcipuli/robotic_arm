// robotic_arm.ino
// 4 rotary encoders controlling 4 servo motors.

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int SERVO_1_MIN = 70; //(0 -> 4096)
const int SERVO_1_MID = 300;
const int SERVO_1_MAX = 480;

const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates

const int SERVO_1_START = 140;
const int SERVO_2_START = 300;
const int SERVO_3_START = 470;
const int SERVO_4_START = 340;


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int nb_of_servos = 4;
int button_factor = 20; // 1 increment of the encoder will increase the servo position by 20
float rotation_factor[nb_of_servos] = {0.8, 0.35, 0.7, 2};

int button_previous_pos[nb_of_servos] = {0, 0, 0, 0};
int button_new_pos[nb_of_servos] = {0, 0, 0, 0};
float current_pos[nb_of_servos] = {SERVO_1_START, SERVO_2_START, SERVO_3_START, SERVO_4_START};
float goal_pos[nb_of_servos] = {SERVO_1_START, SERVO_2_START, SERVO_3_START, SERVO_4_START};

int servo_min[nb_of_servos] = {SERVO_1_MIN, SERVO_1_MIN, SERVO_1_MIN, SERVO_1_MIN};
int servo_max[nb_of_servos] = {SERVO_1_MAX, SERVO_1_MAX, SERVO_1_MAX, SERVO_1_MAX};

int servo_start[nb_of_servos] = {SERVO_1_START, SERVO_2_START, SERVO_3_START, SERVO_4_START};

RotaryEncoder encoder_list[nb_of_servos] = {RotaryEncoder(8 , 9 , RotaryEncoder::LatchMode::TWO03), RotaryEncoder(6 , 7 , RotaryEncoder::LatchMode::TWO03), RotaryEncoder(4 , 5 , RotaryEncoder::LatchMode::TWO03), RotaryEncoder(2 , 3 , RotaryEncoder::LatchMode::TWO03)};

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  for (int i = 0; i < nb_of_servos; i++) {
    pwm.setPWM(i, 0, servo_start[i]);
  }

  delay(2000);
}

void loop()
{

  for (int i = 0; i < nb_of_servos; i++) {

    // For each encoder, get the position
    encoder_list[i].tick();
    button_new_pos[i] = encoder_list[i].getPosition();

    // If the position has changed
    if (button_new_pos[i] != button_previous_pos[i]) {
      // Compute the target/goal position of the servo
      goal_pos[i] = goal_pos[i] + (button_new_pos[i] - button_previous_pos[i]) * button_factor;

      // Check that it is not out of range for the servo
      if (goal_pos[i] > servo_max[i])
        goal_pos[i] = servo_max[i];
      if (goal_pos[i] < servo_min[i])
        goal_pos[i] = servo_min[i];

      // Update the stored button position
      button_previous_pos[i] = button_new_pos[i];
    }

    // Then at each iteration, if the current position of the servo is different from the target one, we decrease the gap
    if (current_pos[i] < goal_pos[i])
      current_pos[i] = current_pos[i] + rotation_factor[i];
    else {
      current_pos[i] = current_pos[i] - rotation_factor[i];
    }
    
    pwm.setPWM(i, 0, current_pos[i]);
  }
}
