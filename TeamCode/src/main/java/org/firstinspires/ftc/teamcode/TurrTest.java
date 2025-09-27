package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
@Config
public class TurrTest extends LinearOpMode {
    private DcMotorEx TurrMotor;
    private DcMotorEx intakeStage1;
    private DcMotorEx intakeStage2;
    public static double pow=1;
    public static boolean powering=false;
    public static double velocity_MperS=2;
    public static double intakePow=0.8;
    private double velocity;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());//telem stuff
        TurrMotor = hardwareMap.get(DcMotorEx.class,"Motor0");
        intakeStage1 = hardwareMap.get(DcMotorEx.class,"Motor1");
        intakeStage2= hardwareMap.get(DcMotorEx.class,"Motor2");
        TurrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//gets encoder stuff
        waitForStart();
        while (opModeIsActive()){
            intakeStage1.setPower(intakePow);//sets intake power
            intakeStage2.setPower(intakePow);//sets intake power
            if(powering) {//when u want to use power instead of velocity
                if (gamepad1.a) {
                    TurrMotor.setPower(pow);//set power when press a
                } else {
                    TurrMotor.setPower(0);// sets to 0 when not pressed
                }
            }else if(!powering){
                if (gamepad1.a) {
                    //diamter of wheel is 72mm so 72pi is circumference and 72pi/1000 to convert to meters
                    //this equals 0.22619467 meters per revolution
                    //then if given like 10 meters/s just do 10/0.22619467 to get revolutions needed per sec then multiply by 29 to convert to ticks
                    double metersPerRev=(velocity_MperS/((72*Math.PI)/1000));
                    velocity=((metersPerRev)*29);
                    TurrMotor.setVelocity(velocity);//max is around 2300 at 13.4 battery
                } else {
                    TurrMotor.setVelocity(0);// sets to 0 when not pressed
                }
            }
            //telemtry
            telemetry.addData("current velocity",TurrMotor.getVelocity());
            telemetry.addData("currTicks: ", TurrMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}