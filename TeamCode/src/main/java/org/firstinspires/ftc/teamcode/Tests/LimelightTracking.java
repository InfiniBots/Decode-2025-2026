package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID.kD;
import static org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID.kI;
import static org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID.kP;

import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/*@Autonomous
public class LimelightTracking extends OpMode{

    public void init(){

    }

    public void loop(){

        if (result == null || !result.isValid()){
            // ok ill add the code later crodie
        } else {
            if (x != wishingX){
                Turret.setPositionPIDFCoefficients();
            }
        }

    }
    private DcMotorEx Turret;
    private Limelight3A limelight;

    private final double kp = 0.0 ;
    private final double ki = 0.0;
    private final double kd = 0.0;
    private final double kf = 0.0;

    private final double wishingX = 0.0;

    LLResult result = limelight.getLatestResult();

    private double x = result.getTx();

    Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

}

 */
