#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
    double kp;
    double ki;
    double kd;

    double max_p_term;
    double max_i_term;
    double max_d_term;

    double integral;
    double previous_error;

    double filter_value;
    double alpha;  // For low-pass filtering
    double min_cmd_threshold;

public:
    PIDController(double kp, double ki, double kd,
                  double max_p = 500.0, double max_i = 300.0, double max_d = 100.0,
                  double alpha = 0.5, double min_cmd = 50.0)
        : kp(kp), ki(ki), kd(kd),
          max_p_term(max_p), max_i_term(max_i), max_d_term(max_d),
          integral(0.0), previous_error(0.0),
          filter_value(0.0), alpha(alpha), min_cmd_threshold(min_cmd) {}

    void reset() {
        integral = 0.0;
        previous_error = 0.0;
        filter_value = 0.0;
    }

    double update(double setpoint, double measured, double dt) {
// Filter the measured signal
        filter_value = alpha * filter_value + (1.0 - alpha) * measured;
        double error = setpoint - filter_value;

        // Integral with clamping
        integral += error * dt;
        integral = constrain(integral, -max_i_term, max_i_term);

        // Derivative
        double derivative = (error - previous_error) / dt;
        previous_error = error;

        // PID terms
        double p_term = constrain(kp * error, -max_p_term, max_p_term);
        double i_term = constrain(ki * integral, -max_i_term, max_i_term);
        double d_term = constrain(kd * derivative, -max_d_term, max_d_term);

        double output = p_term + i_term + d_term;

        // Debug print
        // Serial.print("Error: "); Serial.print(error);
        // Serial.print(" | P: "); Serial.print(p_term);
        // Serial.print(" | I: "); Serial.print(i_term);
        // Serial.print(" | D: "); Serial.print(d_term);
        // Serial.print(" | Output: "); Serial.println(output);

        if (setpoint == 0.0) {
            reset();
            return 0.0;
        }

        // // Ensure minimum command is applied if output is non-zero
        // if (output > 0)
        //     output = max(output, min_cmd_threshold);
        // else if (output < 0)
        //     output = min(output, -min_cmd_threshold);

        return output;
    }

    double getFilteredValue() const {
        return filter_value;
    }

    double getIntegral() const {
        return integral;
    }

    double getPreviousError() const {
        return previous_error;
    }

    void setTunings(double kp_, double ki_, double kd_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    void setOutputLimits(double max_p, double max_i, double max_d) {
        max_p_term = max_p;
        max_i_term = max_i;
        max_d_term = max_d;
    }

    void setFilterAlpha(double a) {
        alpha = a;
    }
};

#endif // PIDCONTROLLER_H
