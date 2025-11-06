package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightTurretTracker {
    private DcMotorEx turret;
    private VoltageSensor Voltage;
    private Limelight3A limelight;
    private long lastTime;
    private double turret_lastError = 0;
    private double turret_errorSum = 0;
    private final double wishingX = 0.00;
    public static double turret_kp = 0.02;
    public static double turret_ki = 0.00000006;
    public static double turret_kd = 0.003;
    public double error;
    public Telemetry telemetry;
    public long curTime;


    public double turret_PID(long curtime) {
        long dtime = curtime - lastTime;
        lastTime = curtime;
        double error = this.error;
        if (dtime <= 0) {
            dtime = 1;
        }
        double errorChange = (error - turret_lastError) / dtime;
        turret_errorSum += (error * dtime);
        turret_lastError = error;
        return ((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)) * (12.0 / Voltage.getVoltage());
    }
    public double power;

    public LimelightTurretTracker(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry=telemetry;
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        Voltage = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        lastTime = System.currentTimeMillis();
    }





    public double update() {
        LLResult result = limelight.getLatestResult();
        curTime = System.currentTimeMillis();
        if (result == null || !result.isValid()) {
            power= 0;
            telemetry.addData("not recognized",1);
        }

        double x = result.getTx();
         error = -x;
        if (x != 0.00) {
            if (result == null || !result.isValid()) {
                boolean isntGettingRecognized = true; // TEMPORARY
            } else {
                if (error != 0.00) {
                    power = turret_PID(curTime);
                }

            }
        }
        telemetry.addData("Turret Power: ", turret.getPower());
        telemetry.addData("error",error);
        telemetry.addData("error: ", error);
        telemetry.addData("currpos",turret.getCurrentPosition());
        telemetry.addData("tx: ", result.getTx());
        telemetry.update();
        return power;
    }
}

