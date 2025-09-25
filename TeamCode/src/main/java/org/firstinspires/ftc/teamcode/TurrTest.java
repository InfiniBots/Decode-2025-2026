package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    public static double vel=0.5;
    public static boolean powering=false;
    public static double tickPerRev=29;
    public static int RPM=1150;
    public static double intakePow=0.8;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TurrMotor = hardwareMap.get(DcMotorEx.class,"Motor0");
        intakeStage1 = hardwareMap.get(DcMotorEx.class,"Motor1");
        intakeStage2= hardwareMap.get(DcMotorEx.class,"Motor2");
        TurrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            intakeStage1.setPower(intakePow);
            intakeStage2.setPower(intakePow);
            if(powering) {
                if (gamepad1.a) {
                    TurrMotor.setPower(pow);
                } else {
                    TurrMotor.setPower(0);
                }
            }else if(!powering){
                if (gamepad1.a) {
                    TurrMotor.setVelocity(tickPerRev*RPM*vel);
                } else {
                    TurrMotor.setVelocity(0);
                }
            }
            telemetry.addData("current velocity",TurrMotor.getVelocity());
            telemetry.update();
        }
    }
}