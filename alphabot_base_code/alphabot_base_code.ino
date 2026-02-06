#define BOARD_NANO
// #define BOARD_ESP32
#define DEBUG 0

#include "config.h"
#include "odom.h"
#include <PID_v1.h>
#include "measureVoltage.h"
#include "bumper.h"
#include <stdlib.h>

// Encoder direction
String right_wheel_sign = "p";
String left_wheel_sign = "p";

// Timing
unsigned long last_millis = 0;
const unsigned long interval = 1000 / LOOP_FREQ;

// Serial command parsing
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
char current_param_cmd = '\0'; // 'k','i','d','m','f','s'
float voltage_reading  = 12.9;

// Safety timeout mechanism
unsigned long last_valid_command = 0;
const unsigned long COMMAND_TIMEOUT_MS = 5000;  // 500ms safety timeout

// PID variables
double right_wheel_cmd_vel = 0.0, left_wheel_cmd_vel = 0.0;
double right_wheel_meas_vel = 0.0, left_wheel_meas_vel = 0.0;
double right_wheel_filtered_vel = 0.0, left_wheel_filtered_vel = 0.0;
double right_wheel_cmd = 0.0, left_wheel_cmd = 0.0;

// PID Gains (tunable at runtime)
double Kp = 70.0, Ki = 20.5, Kd = 0.1;

// PID control variables
double right_input, right_output, right_setpoint;
double left_input, left_output, left_setpoint;

PID rightPID(&right_input, &right_output, &right_setpoint, Kp, Ki, Kd, DIRECT);
PID leftPID(&left_input, &left_output, &left_setpoint, Kp, Ki, Kd, DIRECT);

// Motor correction scale
double left_scale = 1.0;
double right_scale = 0.98;

// Filtering
const double alpha = 0.9;

// Low-speed compensation (to overcome static friction)
double min_pwm = 30.0;            // Minimum PWM when setpoint > 0
double feedforward_pwm = 15.0;    // Extra PWM for low-speed start
double low_speed_threshold = 0.06; // m/s threshold for applying feedforward

measureVoltage batteryMonitor;

void setup() {

  batteryMonitor.init(voltage_pin);
  batteryMonitor.setBatteryType("12V_LEAD_ACID");
  setupBumper();
  // pinMode(L298N_enA, OUTPUT);
  // pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // digitalWrite(L298N_in1, HIGH);
  // digitalWrite(L298N_in2, LOW);
  // digitalWrite(L298N_in3, HIGH);
  // digitalWrite(L298N_in4, LOW);

  Serial.begin(115200);

  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);

  // Initialize PID
  rightPID.SetOutputLimits(0, 255);
  leftPID.SetOutputLimits(0, 255);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);
}

void loop() {
  if (Serial.available()) {
    char chr = Serial.read();

    if(chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
      current_param_cmd = '\0';
    }
    else if(chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
      current_param_cmd = '\0';
    }
    else if(chr == 'p') {
      if(is_right_wheel_cmd && !is_right_wheel_forward) {
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward) {
        is_left_wheel_forward = true;
      }
    }
    else if(chr == 'n') {
      if(is_right_wheel_cmd && is_right_wheel_forward) {
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward) {
        is_left_wheel_forward = false;
      }
    }
    else if(chr == 'k' || chr == 'i' || chr == 'd' || chr == 'm' || chr == 'f' || chr == 's') {
      // Parameter commands:
      // kXX, -> Kp
      // iXX, -> Ki
      // dXX, -> Kd
      // mXX, -> min_pwm
      // fXX, -> feedforward_pwm
      // sXX, -> low_speed_threshold (m/s)
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
      current_param_cmd = chr;
    }
    else if(chr == ',') {
      if(is_right_wheel_cmd) {
        // Safe string-to-float conversion with validation
        char* end_ptr;
        float parsed_val = (float)strtod(value, &end_ptr);
        if (end_ptr != value && parsed_val >= 0.0) {
          right_wheel_cmd_vel = parsed_val;
          last_valid_command = millis();  // Update timestamp on valid command
        }
      }
      else if(is_left_wheel_cmd) {
        // Safe string-to-float conversion with validation
        char* end_ptr;
        float parsed_val = (float)strtod(value, &end_ptr);
        if (end_ptr != value && parsed_val >= 0.0) {
          left_wheel_cmd_vel = parsed_val;
          is_cmd_complete = true;
          last_valid_command = millis();  // Update timestamp on valid command
        }
      }
      else if(current_param_cmd != '\0') {
        // Parse parameter commands
        char* end_ptr;
        float parsed_val = (float)strtod(value, &end_ptr);
        if (end_ptr != value) {
          if (current_param_cmd == 'k') {
            Kp = parsed_val;
          } else if (current_param_cmd == 'i') {
            Ki = parsed_val;
          } else if (current_param_cmd == 'd') {
            Kd = parsed_val;
          } else if (current_param_cmd == 'm') {
            min_pwm = constrain(parsed_val, 0.0, 255.0);
          } else if (current_param_cmd == 'f') {
            feedforward_pwm = constrain(parsed_val, 0.0, 255.0);
          } else if (current_param_cmd == 's') {
            low_speed_threshold = max(parsed_val, 0.0f);
          }
          rightPID.SetTunings(Kp, Ki, Kd);
          leftPID.SetTunings(Kp, Ki, Kd);
          last_valid_command = millis();
        }
      }
      value_idx = 0;
      strncpy(value, "00.00", sizeof(value) - 1);  // Safe string copy
      value[sizeof(value) - 1] = '\0';
      current_param_cmd = '\0';
    }
    else {
      if(value_idx < sizeof(value) - 1) {  // Leave room for null terminator
        value[value_idx] = chr;
        value_idx++;
        value[value_idx] = '\0';  // Ensure null termination
      }
    }
  }

  // Safety timeout: stop motors if no valid command received
  unsigned long now = millis();
  if (now - last_valid_command > COMMAND_TIMEOUT_MS) {
    right_wheel_cmd_vel = 0.0;
    left_wheel_cmd_vel = 0.0;
  }

  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval) {
    double dt = (double)interval / 1000.0;
    voltage_reading = batteryMonitor.get_battery_level();
    float percent = batteryMonitor.get_battery_percent();
    compute_pose();

    right_wheel_meas_vel = (LOOP_FREQ * delta_right_tick * 60.0 / TICK_PER_REVOLUT) * 0.10472;
    left_wheel_meas_vel = (LOOP_FREQ * delta_left_tick * 60.0 / TICK_PER_REVOLUT) * 0.10472; 

    // Filter the measured velocities
    right_wheel_filtered_vel = alpha * right_wheel_filtered_vel + (1.0 - alpha) * abs(right_wheel_meas_vel);
    left_wheel_filtered_vel = alpha * left_wheel_filtered_vel + (1.0 - alpha) * abs(left_wheel_meas_vel);

    // Set PID variables
    right_input = right_wheel_filtered_vel;
    right_setpoint = abs(right_wheel_cmd_vel);
    left_input = left_wheel_filtered_vel;
    left_setpoint = abs(left_wheel_cmd_vel);

    // Run PID
    rightPID.Compute();
    leftPID.Compute();

    // Apply minimum PWM and low-speed feedforward compensation
    double right_ff = (right_setpoint > 0.0 && right_setpoint < low_speed_threshold) ? feedforward_pwm : 0.0;
    double left_ff = (left_setpoint > 0.0 && left_setpoint < low_speed_threshold) ? feedforward_pwm : 0.0;

    if (right_setpoint == 0.0) {
      right_wheel_cmd = 0.0;
    } else {
      double cmd = right_output + right_ff;
      if (right_setpoint >= low_speed_threshold) {
        cmd = max(cmd, min_pwm);
      }
      right_wheel_cmd = constrain(cmd, 0.0, 255.0);
    }

    if (left_setpoint == 0.0) {
      left_wheel_cmd = 0.0;
    } else {
      double cmd = left_output + left_ff;
      if (left_setpoint >= low_speed_threshold) {
        cmd = max(cmd, min_pwm);
      }
      left_wheel_cmd = constrain(cmd, 0.0, 255.0);
    }


    // Check bumper sensors
    int bumperStatus = checkBumper();
    // Output debug or telemetry
    String encoder_read;
    if(DEBUG) {
      encoder_read = "r:" + String(right_input) + "," + String(right_output) +
                     ",l:" + String(left_input) + "," + String(left_output) + ",";
    } else {
      float rv = abs(right_wheel_meas_vel);
      float lv = abs(left_wheel_meas_vel);
      if (rv == 0.0) rv = 0.0;
      if (lv == 0.0) lv = 0.0;
      encoder_read = "r" + right_wheel_sign + String(rv, 2) +
             ",l" + left_wheel_sign + String(lv, 2) + // ",";
             ",v"+ "p" +String(percent,2) +
             ",b" + "p" + String(bumperStatus)+ ",";
    }

    Serial.println(encoder_read);

    moveLeftMotor();
    moveRightMotor();
    // Serial.print(right_wheel_cmd);Serial.print("__");Serial.println(left_wheel_cmd);
    last_millis = current_millis;
  }
}

void moveRightMotor(){
  if(is_right_wheel_forward == true){
    analogWrite(L298N_in1, right_wheel_cmd);
    analogWrite(L298N_in2, 0);
  }else{
    analogWrite(L298N_in2, right_wheel_cmd);
    analogWrite(L298N_in1, 0);
  }
}

void moveLeftMotor(){
  if(is_left_wheel_forward == true){
    analogWrite(L298N_in3, left_wheel_cmd);
    analogWrite(L298N_in4, 0);
  }else{
    analogWrite(L298N_in4, left_wheel_cmd);
    analogWrite(L298N_in3, 0);
  }
}


void rightEncoderCallback() {
  if(digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = "p";
    right_encoder_counter++;
  } else {
    right_wheel_sign = "n";
    right_encoder_counter--;
  }
}

void leftEncoderCallback() {
  // NORMALIZED: Left encoder now uses same logic as right for consistency
  if(digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = "n";
    left_encoder_counter--;
  } else {
    left_wheel_sign = "p";
    left_encoder_counter++;
  }
}
