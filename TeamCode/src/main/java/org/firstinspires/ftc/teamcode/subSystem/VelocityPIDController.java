package org.firstinspires.ftc.teamcode.subSystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VelocityPIDController {
    public static double kp = 0.002;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0;
    private double lastError = 0;
    private double errorSum = 0;

    public double PID(double currentVelocity, double targetVelocity, long time) {
        double error = targetVelocity - currentVelocity;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - lastError) / time;
        errorSum += (error * time);
        lastError = error;
        return ((kp * error) + (ki * errorSum) + (kd * errorChange) + ((0.0007448464-(3.3333219e-7*targetVelocity)+(8.791839e-11*targetVelocity*targetVelocity)) * targetVelocity));
    }
}
