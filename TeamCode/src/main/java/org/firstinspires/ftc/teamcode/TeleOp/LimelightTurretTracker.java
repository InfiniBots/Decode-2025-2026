package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class LimelightTurretTracker {
    private DcMotorEx turret;
    private VoltageSensor voltage;
    private Limelight3A limelight;
    private long lastTime;
    private double turret_lastError = 0;
    private double turret_errorSum = 0;
    private final double wishingX = 0.00;
    public static double turret_kp = 0.02;
    public static double turret_ki = 0.00000006;
    public static double turret_kd = 0.003;

    public LimelightTurretTracker(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        voltage = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        lastTime = System.currentTimeMillis();
    }

    public void update() {
        long curTime = System.currentTimeMillis();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turret.setPower(0);
            return;
        }

        double x = result.getTx();
        double error = wishingX - x;

        if (x != 0.00) {
            double dtime = curTime - lastTime;
            lastTime = curTime;
            if (dtime <= 0) dtime = 1;

            double errorChange = (error - turret_lastError) / dtime;
            turret_errorSum += (error * dtime);
            turret_lastError = error;

            double power = ((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)) * (12.0 / voltage.getVoltage());
            turret.setPower(power);
        }
    }
}