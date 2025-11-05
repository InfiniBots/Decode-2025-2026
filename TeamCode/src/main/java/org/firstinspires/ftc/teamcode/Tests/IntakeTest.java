package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@Config
@TeleOp(group = "Test")
public class IntakeTest extends LinearOpMode {
    private DcMotorEx Intake;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake = hardwareMap.get(DcMotorEx.class,"Intake");

        waitForStart();
        while(opModeIsActive()) {
            Intake.setPower(gamepad1.left_stick_y);
        }
    }
}
