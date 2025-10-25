package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.hardware.impl.MotorEx;
@TeleOp
@Config
public class turretMover extends LinearOpMode {
    private DcMotorEx Turret;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret = hardwareMap.get(DcMotorEx.class,"Turret");
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                Turret.setPower(0.5);
            }
            if(gamepad1.b){
                Turret.setPower(-0.5);
            }
        }
    }
}
