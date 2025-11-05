package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@Disabled
public class Drive{
    private DcMotorEx FrontRight;
    private DcMotorEx FrontLeft;
    private DcMotorEx BackRight;
    private DcMotorEx BackLeft;
    public double FR=0.0;
    public double FL=0.0;
    public double BR=0.0;
    public double BL=0.0;
    public static double strafingOffest=1.1;
    public static double rotationAdjustment=0.8;
    public Drive(LinearOpMode op){
        FrontRight = op.hardwareMap.get(DcMotorEx.class,"rightFront");
        FrontLeft = op.hardwareMap.get(DcMotorEx.class,"leftFront");
        BackRight = op.hardwareMap.get(DcMotorEx.class,"rightRear");
        BackLeft = op.hardwareMap.get(DcMotorEx.class,"leftRear");


        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void driveInputs(double x, double y, double theta){
        x = x * strafingOffest;
        theta=theta * rotationAdjustment;
        FR = (y-x-theta);
        BR = (y+x-theta);
        FL = (y+x+theta);
        BL = (y-x+theta);
        double max = Math.max(Math.abs(FR),Math.max(Math.abs(BR),Math.max(Math.abs(FL),Math.abs(BL))));
        if (max>1){
            FR = FR/max;
            BR = BR/max;
            FL = FL/max;
            BL = BL/max;
        }
        FrontRight.setPower(FR);
        FrontLeft.setPower(FL);
        BackRight.setPower(BR);
        BackLeft.setPower(BL);
    }
}