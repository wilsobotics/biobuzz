package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDf {
    ElapsedTime time = new ElapsedTime();
    double previous_time;
    double kP, kI, kD, kV, kA, kS, kG;

    public PIDf(double P, double I, double D, double V, double A, double S, double G) {
        kP = P; kI = I; kD = D; kV = V; kA = A; kS = S; kG = G;
        time.reset();
        previous_time = time.time();
    }





    public void PID(Double current_state, Double desired_state) {
        double current_time = time.time();
        double current_error = desired_state - current_state;

        double p = kP * current_error

        i += k_i * (current_error * (current_time - previous_time))

        if i > max_i:
        i = max_i
        elif i <-max_i:
        i = -max_i

        D = k_d * (current_error - previous_error) / (current_time - previous_time)

        output = p + i + d

        previous_error = current_error
        previous_time = current_time
    }
}
