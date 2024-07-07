#ifndef ALPHABOT_FIRMWARE_MODEL_H
#define ALPHABOT_FIRMWARE_MODEL_H


class Model{
  public:
    Model();
    // void robot_state_update();
    double get_left_angle();
    double get_right_angle();
    void computer_position();
    double get_left_velocity();
    double get_right_velocity();

    void update_encoder(long left, long right, double delat_time);
  
  private:
    volatile long tick_left = 0;
    volatile long tick_right = 0;

    double left_current_pos = 0;
    double right_current_pos = 0;

    double left_prevouse_pos = 0;
    double right_prevouse_pos = 0;

    float WHEEL_DIAMETER = 70/1000; //m
    float WHEEL_SEPERATION = 236/1000; //m
    int TICK_PER_REVOLUTION = 231; 

    // cm traveled each gear tick
    const double DIST_PER_TICK = (WHEEL_DIAMETER * 3.14) / TICK_PER_REVOLUTION;

};

#endif
