package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Prematch extends LinearOpMode {
private DcMotorEx TurrMotor;
private DcMotorEx TurrMotor2;
private Servo Stopper1;//non cooked
private Servo Stopper2;//limelight side
public static boolean flywheelCheck = false;
public static boolean nonllServo=false;
public static boolean llServo=false;
public static boolean open1=false;
public static boolean open2=false;

    @Override
    public void runOpMode() throws InterruptedException {
        TurrMotor = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        TurrMotor2 = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
        TurrMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
        Stopper2 = hardwareMap.get(Servo.class, "Stopper2");
        waitForStart();
        while(opModeIsActive()){
            //flywheels
            if(flywheelCheck){
                TurrMotor.setPower(0.5);
                TurrMotor2.setPower(0.5);
                telemetry.addData("Flywheels", "On");
                if(Math.abs(TurrMotor.getVelocity())>67){
                    telemetry.addData("top flywheel working",true);
                }else{
                    telemetry.addData("top flywheel working",false);
                }
                if(Math.abs(TurrMotor2.getVelocity())>67){
                    telemetry.addData("bottom flywheel working",true);
                }else{
                    telemetry.addData("bottom flywheel working",false);
                }
            }
            else{
                TurrMotor.setPower(0);
                TurrMotor2.setPower(0);
            }

            //stoppers
            if(gamepad1.a||nonllServo) {
                if(open1){
                    Stopper1.setPosition(0.767);
                }else{
                    Stopper1.setPosition(1);

                }
            }
            if(gamepad1.b||llServo){
                if(open2){
                    Stopper2.setPosition(0.62);
                }else{
                    Stopper2.setPosition(1);
                }
            }
            //someone add turret and maybe some other stuff idk




            telemetry.update();

        }
    }
}
