package org.firstinspires.ftc.teamcode.Tests;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous
public class LimelightTracking extends OpMode{
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    private double targetPosition = 0;
    private boolean isntGettingRecognized = false;
    private Limelight3A limelight;
    private long lastTime;
    private long curTime;
    public static double turret_kp = 0.02;
    public static double turret_ki = 0.00000006;
    public static double turret_kd = 0.003;
    private double turret_lastError = 0;
    private double turret_errorSum = 0;
    public static double ticksPRotation = 2288;
    public double ticksPerAng = (ticksPRotation / 360.0);
    private final double wishingX = 0.00;
    public LLResult result;
    public double error;

    public double turret_PID(long curtime) {
        long dtime=curtime-lastTime;
        lastTime=curtime;
        double error =this.error;
        if (dtime <= 0) {
            dtime = 1;
        }
        double errorChange = (error - turret_lastError) / dtime;
        turret_errorSum += (error * dtime);
        turret_lastError = error;
        return ((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)) * (12.0 / Voltage.getVoltage());
    }

    public void init(){


        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);

  }

    public void start(){
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        lastTime = System.currentTimeMillis();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Voltage = hardwareMap.voltageSensor.iterator().next();
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void loop(){
        curTime=System.currentTimeMillis();
        result = limelight.getLatestResult();
        double x = result.getTx();
        error = wishingX - x;

        if (result == null || !result.isValid()){
            isntGettingRecognized = true; // TEMPORARY
        } else {
            if (x != 0.00){
                double power = turret_PID(curTime);
                Turret.setPower(power);
            }


            telemetry.addData("isn'tGettingRecognized: ", isntGettingRecognized);
            telemetry.addData("target x: ", x);
            telemetry.addData("error: ", error);
            telemetry.addData("currpos",Turret.getCurrentPosition());
            telemetry.update();
        }

    }



}


