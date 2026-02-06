struct Coordinate {
public:
  double x;
  double y;
  double theta;

  Coordinate(double _x, double _y, double _theta = 0.0)
  {
    x = _x;
    y = _y;
    theta = _theta;
  }
};


const double CLICK_PER_ROTATION = 990.0;
const double WHEEL_DIAMETER = 0.102;
const double DISTANCE_PER_TICK = (WHEEL_DIAMETER * PI)/ CLICK_PER_ROTATION;
const double WHEEL_SEPERATION = 0.212;

// Encoders
volatile long right_encoder_counter = 0;
volatile long left_encoder_counter = 0;

volatile long previous_right_tick = 0.0;
volatile long previous_left_tick = 0.0;
volatile long delta_right_tick = 0.0;
volatile long delta_left_tick = 0.0;


// distance traveled by wheel in cm
double distance_right = 0.0, distance_left = 0.0;

// previous distance traveled
double previous_distance_right = 0.0, previous_distance_left = 0.0;
double delta_distance_right = 0.0, delta_distance_left = 0.0;
double delta_distance_total = 0.0;

double right_velocity = 0.0, left_velocity = 0.0;
Coordinate pose(0.0,0.0,0.0);
Coordinate deltaPose(0.0,0.0,0.0);

void compute_pose()
{
  // Disable interrupts to safely read volatile encoder counters
  noInterrupts();
  delta_right_tick = right_encoder_counter - previous_right_tick;
  delta_left_tick = left_encoder_counter - previous_left_tick;
  previous_left_tick = left_encoder_counter;
  previous_right_tick = right_encoder_counter;
  interrupts();

  distance_right += delta_right_tick * DISTANCE_PER_TICK;
  distance_left  += delta_left_tick * DISTANCE_PER_TICK;

  delta_distance_right = distance_right - previous_distance_right;
  delta_distance_left = distance_left - previous_distance_left;

  previous_distance_right = distance_right;
  previous_distance_left = distance_left;

  delta_distance_total = (delta_distance_right + delta_distance_left) / 2.0;

  deltaPose.theta = (delta_distance_right - delta_distance_left) / WHEEL_SEPERATION;

  deltaPose.x = delta_distance_total * cos(pose.theta + deltaPose.theta / 2);
  deltaPose.y = delta_distance_total * sin(pose.theta + deltaPose.theta / 2);

  // update coordinates
  pose.x += deltaPose.x;
  pose.y += deltaPose.y;
  pose.theta += deltaPose.theta;

  // Serial.print(pose.x);
  // Serial.print(",");
  // Serial.print(pose.y);
  // Serial.print(",");
  // Serial.println(pose.theta*180/PI);
}

void compute_velocity(double intravel)
{
  right_velocity = (LOOP_FREQ * delta_right_tick * (60.0 / TICK_PER_REVOLUT)) * 0.10472;
  left_velocity = (LOOP_FREQ * delta_left_tick * (60.0 / TICK_PER_REVOLUT)) * 0.10472;

  Serial.print(right_velocity);
  Serial.print(",");
  Serial.println(left_velocity);
}
