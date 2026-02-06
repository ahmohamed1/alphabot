#define BOARD_NANO
// #define BOARD_ESP32
#define DEBUG 0

#include "config.h"
#include "odom.h"
#include <PID_v1.h>
#include "measureVoltage.h"
#include "bumper.h"

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
float voltage_reading  = 12.9;

// PID variables
double right_wheel_cmd_vel = 0.0, left_wheel_cmd_vel = 0.0;
double right_wheel_meas_vel = 0.0, left_wheel_meas_vel = 0.0;
double right_wheel_filtered_vel = 0.0, left_wheel_filtered_vel = 0.0;
double right_wheel_cmd = 0.0, left_wheel_cmd = 0.0;

// PID Gains
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
  rightPID.SetOutputLimits(20, 255);
  leftPID.SetOutputLimits(20, 255);
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
    }
    else if(chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
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
    else if(chr == ',') {
      if(is_right_wheel_cmd) {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      strcpy(value, "00.00");
    }
    else {
      if(value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
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

    // Apply minimum PWM threshold if needed
    right_wheel_cmd = (right_setpoint == 0.0) ? 0.0 : max(right_output, 20.0);
    left_wheel_cmd = (left_setpoint == 0.0) ? 0.0 : max(left_output, 20.0);


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
  if(digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = "n";
    left_encoder_counter--;
  } else {
    left_wheel_sign = "p";
    left_encoder_counter++;
  }
}
