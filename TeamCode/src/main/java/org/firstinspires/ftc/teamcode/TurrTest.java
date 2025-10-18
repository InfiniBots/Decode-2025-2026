package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
@Config
public class TurrTest extends LinearOpMode {
    private DcMotorEx TurrMotor;
    private DcMotorEx intakeStage1;
    private DcMotorEx intakeStage2;
    private DcMotorEx TurrMotor2;
    public static double powerinpow=1;
    public static double pow=1;
    public static boolean powering=false;
    public static int ticksPSec;
    public static double intakePow=1;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());//telem stuff
        TurrMotor2 = hardwareMap.get(DcMotorEx.class,"Motor1");
        TurrMotor = hardwareMap.get(DcMotorEx.class,"Motor0");
        //intakeStage1 = hardwareMap.get(DcMotorEx.class,"Motor0");
        //intakeStage2= hardwareMap.get(DcMotorEx.class,"Motor1");
        TurrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurrMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
           /* if(gamepad1.b){
                intakeStage1.setPower(intakePow);//sets intake power
                intakeStage2.setPower(intakePow);//sets intake power
            }else{
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);
            }*/

            if(powering) {//when u want to use power instead of velocity
                if (gamepad1.a) {
                    TurrMotor.setPower(powerinpow);
                    TurrMotor2.setPower(powerinpow);
                } else {
                    TurrMotor.setPower(0);// sets to 0 when not pressed
                    TurrMotor2.setPower(0);
                }
            }else if(!powering){
                if (gamepad1.a) { //castrate gautam
                    TurrMotor.setVelocity(ticksPSec);
                    TurrMotor2.setVelocity(ticksPSec);
                } else {
                    TurrMotor.setVelocity(0);// sets to 0 when not pressed
                    TurrMotor2.setVelocity(0);// sets to 0 when not pressed
                }
            }
            //telemtry
            telemetry.addData("TurrMotor velocity",TurrMotor.getVelocity());
            telemetry.addData("TurrMotor2 velocity", TurrMotor2.getVelocity());
            telemetry.addData("Target speed",ticksPSec);
            telemetry.addData("Voltage",TurrMotor.getCurrent(CurrentUnit.AMPS)+TurrMotor2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("power",TurrMotor.getPower());
            PanelsTelemetry.INSTANCE.getTelemetry().addData("hi",1);
            PanelsTelemetry.INSTANCE.getTelemetry().update();
            telemetry.update();
        }
    }
}