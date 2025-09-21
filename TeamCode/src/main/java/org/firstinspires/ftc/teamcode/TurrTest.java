package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
@Config
public class TurrTest extends LinearOpMode {
    private DcMotorEx TurrMotor;
    public static double pow=1;
    public static double vel=0.5;
    public static boolean powering=true;
    public static double tickPerRev=29;
    public static int RPM=1150;

    @Override
    public void runOpMode() {
        TurrMotor = hardwareMap.get(DcMotorEx.class,"TurrMotor");
        TurrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
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
        }
    }
}
