package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Configurable
@TeleOp
public class Tele extends LinearOpMode {
    private Robot robot;
    public Telemetry telemetry;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot=new Robot(this, telemetry);

        waitForStart();
        while(opModeIsActive()){
            robot.drive.driveInputs(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);//drive
            if(gamepad1.right_trigger>0.25){//intake
                robot.intakingApproval=true;
            }else {
                robot.intakingApproval=false;
            }
            if(gamepad1.a){//shoot
                robot.Mode="shooting";
            }else{
                robot.Mode="Driving";
            }
            robot.UpdateRobot();
            telemetry.update();
        }
    }
}
