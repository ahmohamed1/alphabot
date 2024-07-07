#include "alphabot_firmware/base_model.h"


void Model::update_encoder(long left, long right, double delta_time)
{
    tick_left = left;
    tick_right = right;
    Model::computer_position(delat_time);
}

double Model::get_left_angle()
{
    return left_current_pos;
}

double Model::get_right_angle()
{
    return right_current_pos;
}


double Model::get_left_velocity()
{
    return left_velocity;
}

double Model::get_right_velocity()
{
    return right_velocity;
}

void Model::computer_position(double delat_time)
{
    left_current_pos = tick_left * TICK_PER_REVOLUTION;
    right_current_pos = tick_right * TICK_PER_REVOLUTION;

    left_velocity = (left_current_pos - left_prevouse_pos) / delat_time;
    right_velocity = (right_current_pos - right_prevouse_pos) / delat_time;

    left_prevouse_pos = left_current_pos;
    right_prevouse_pos = right_current_pos;
}



// void Model::robot_state_update(long tick_left, long tick_right)
// {
   
//     // 1: update the distance
//     Dl += (tick_left - old_tick_left) * DIST_PER_TICK;
//     Dr += (tick_right - old_tick_right) * DIST_PER_TICK;

//     // 2: Get the delta distance
//     float delta_distance_r = Dr - old_Dr;
//     float delta_distance_l = Dl - old_Dl;

//     // 3: compute coordinate of the robot
//     float delta_distance_center = (delta_distance_l + delta_distance_r)/2.0;
//     float delta_heading = (delta_distance_r - delta_distance_l)/B;

//     float delta_x = delta_distance_center * cos(pose1.heading + delta_heading / 2.0);
//     float delta_y = delta_distance_center * sin(pose1.heading + delta_heading / 2.0);
    
//     pose1.x += delta_x;
//     pose1.y += delta_y;
//     pose1.heading += delta_heading;
//     // 5: assigne the new value to old value
//     old_tick_left = tick_left;
//     old_tick_right = tick_right;
//     old_Dr = Dr;
//     old_Dl = Dl;
        
// }

