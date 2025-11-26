package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystem.LimelightTracking;

@TeleOp(group = "Test")
@Config
public class turretMover extends LinearOpMode {
    private DcMotorEx Turret;
    private Limelight3A limelight;
    private LLResult llResult;
    private VoltageSensor Voltage;
    private LimelightTracking lltracking;
    private long curTime;
    private long deltaTime;
    private long lastTime;
    public static boolean turrOn=false;

    public static double turret_kp = 0;
    public static double turret_ki = 0;
    public static double turret_kd = 0;
    private double turret_lastError = 0;
    private double power;

    private double turret_errorSum = 0;
    private long turret_lastTime;
    public static int turret_target=0;
    public static double ticksPRotation = 2288;
    public double ticksPerAng = (ticksPRotation / 360.0);


    public double turret_PID(double currPos, double targetPos, long curtime) {
        long dtime=curtime-lastTime;
        lastTime=curtime;
        double error = targetPos - currPos;
        if (dtime <= 0) {
            dtime = 1;
        }
        double errorChange = (error - turret_lastError) / dtime;
        turret_errorSum += (error * dtime);
        turret_lastError = error;
        return ((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)) * (12.0 / Voltage.getVoltage());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        lltracking = new LimelightTracking(this, telemetry);
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Voltage = hardwareMap.voltageSensor.iterator().next();
        limelight.pipelineSwitch(1);
        lastTime = System.currentTimeMillis();
        waitForStart();
        while (opModeIsActive()) {
            curTime = System.currentTimeMillis();
            lltracking.updateTurret(0,0,0,0,0, 0, true);
            if(gamepad1.a){
                Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            telemetry.addData("Turr pos",Turret.getCurrentPosition());
            telemetry.addData("target",turret_target);
            telemetry.addData("power recieved",Turret.getPower());
            telemetry.addData("Voltage",Voltage.getVoltage());
            telemetry.update();
        }
    }
}
