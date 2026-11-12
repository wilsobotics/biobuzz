package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDf {
    ElapsedTime time = new ElapsedTime();
    public double previous_time, previous_error, i, max_i;
    double kP, kI, kD, kV, kA, kS, kG;

    public PIDf(double P, double I, double D, double V, double A, double S) {
        kP = P;
        kI = I;
        kD = D;
        kV = V;
        kA = A;
        kS = S;
        time.reset();
        previous_time = time.time();
        i = 0;
        max_i = 0;
    }



    public Double PID(Double current_state, Double desired_state) {
        double current_time = time.time();
        double current_error = desired_state - current_state;

        double p = kP * current_error;

        i += kI * (current_error * (current_time - previous_time));

        if (i > max_i) {
            i = max_i;
        }
        if (i <-max_i) {
            i = -max_i;
        }

        double d = kD * (current_error - previous_error) / (current_time - previous_time);
        previous_error = current_error;
        previous_time = current_time;
        return p + i + d;
    }

    public Double FF(Double desired_velocity, Double desired_acceleration) {
        return kV*desired_velocity + kA*desired_acceleration + kS;
    }

    public Double output(Double current_state, Double desired_state) {
        return PID(current_state, desired_state) + FF(current_state, desired_state);
    }
}
