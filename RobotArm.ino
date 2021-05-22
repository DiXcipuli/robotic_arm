#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int SERVO_1_MIN = 70; //(0 -> 4096)
const int SERVO_1_MID = 300;
const int SERVO_1_MAX = 480;
const int SERVO_2_MIN = 70;
const int SERVO_2_MID = 300;
const int SERVO_2_MAX = 480;
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//VARIABLES
const int nb_of_servos = 4;
int button_factor = 20;
float rotation_factor[nb_of_servos] = {0.8, 0.35, 0.7, 2};
int rotation_speed = 20;

int button_previous_pos[nb_of_servos] = {0, 0, 0, 0};
int button_new_pos[nb_of_servos] = {0, 0, 0, 0};
float current_pos[nb_of_servos] = {SERVO_1_MID, SERVO_1_MID, SERVO_1_MID, SERVO_2_MID};
float goal_pos[nb_of_servos] = {SERVO_1_MID, SERVO_1_MID, SERVO_1_MID, SERVO_2_MID};

int servo_min[nb_of_servos] = {SERVO_1_MIN, SERVO_1_MIN, SERVO_1_MIN, SERVO_2_MIN};
int servo_mid[nb_of_servos] = {SERVO_1_MID, SERVO_1_MID, SERVO_1_MID, SERVO_2_MID};
int servo_max[nb_of_servos] = {SERVO_1_MAX, SERVO_1_MAX, SERVO_1_MAX, SERVO_2_MAX};

RotaryEncoder encoder_list[nb_of_servos] = {RotaryEncoder(8 , 9 , RotaryEncoder::LatchMode::TWO03), RotaryEncoder(6 , 7 , RotaryEncoder::LatchMode::TWO03), RotaryEncoder(4 , 5 , RotaryEncoder::LatchMode::TWO03), RotaryEncoder(2 , 3 , RotaryEncoder::LatchMode::TWO03)};

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  for (int i = 0; i < nb_of_servos; i++) {
    pwm.setPWM(i, 0, servo_mid[i]);
  }

  delay(2000);
}

void loop()
{

  for (int i = 0; i < nb_of_servos; i++) {
    encoder_list[i].tick();
    button_new_pos[i] = encoder_list[i].getPosition();
    //Serial.println(button_new_pos[i]);

    if (button_new_pos[i] != button_previous_pos[i]) {
      goal_pos[i] = goal_pos[i] + (button_new_pos[i] - button_previous_pos[i]) * button_factor;

      if (goal_pos[i] > servo_max[i])
        goal_pos[i] = servo_max[i];
      if (goal_pos[i] < servo_min[i])
        goal_pos[i] = servo_min[i];
    button_previous_pos[i] = button_new_pos[i];
    }


    if (current_pos[i] < goal_pos[i])
      current_pos[i] = current_pos[i] + rotation_factor[i];
    else {
      current_pos[i] = current_pos[i] - rotation_factor[i];
    }
     Serial.println(current_pos[1]);
      Serial.println(goal_pos[1]);
    
    pwm.setPWM(i, 0, current_pos[i]);

  }

  //delay(rotation_speed);

}
