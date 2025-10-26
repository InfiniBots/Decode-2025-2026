package org.firstinspires.ftc.teamcode.Tests;



import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous
public class LimelightTracking extends OpMode{
    private DcMotorEx Turret;

    private final double kp = 0.0 ;
    private final double ki = 0.0;
    private final double kd = 0.0;
    private final double kf = 0.0;

    private double targetPosition = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private double lastTime = 0;
    private boolean isntGettingRecognized = false;
    private Limelight3A limelight;



    private final double wishingX = 0.0;



    public void init(){


        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }

    public void start(){
        lastTime = getRuntime();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void loop(){
        LLResult result = limelight.getLatestResult();
        double x = result.getTx();
        double currTime = getRuntime();
        double deltaTime = currTime - lastTime;
        double error = wishingX - x;

        if (deltaTime <= 0){
            deltaTime = 0.0001; // prevents ^ing
        }

        if (result == null || !result.isValid()){
            isntGettingRecognized = true; // TEMPORARY
        } else {
            if (x != wishingX){
                double derivative = (error - lastError) / deltaTime;
                integralSum += error * deltaTime;

                double output = (kp * error) + (ki * integralSum) + (kd * derivative) + (kf * Math.signum(error));

                Turret.setPower(output);

                lastError = error;
                lastTime = currTime;


            }


            telemetry.addData("isn'tGettingRecognized: ", isntGettingRecognized);
            telemetry.addData("target x: ", x);
            telemetry.addData("error: ", error);
        }

    }



}


