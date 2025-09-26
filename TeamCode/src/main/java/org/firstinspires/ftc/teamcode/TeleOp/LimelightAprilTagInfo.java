package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp

public class LimelightAprilTagInfo extends OpMode {
    private Limelight3A vision;
    private IMU imu;
    public void init() {
        vision = hardwareMap.get(Limelight3A.class, "vision");
        vision.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        vision.updateRobotOrientation(orientation.getYaw());
        LLResult vresult = vision.getLatestResult();
        if (vresult != null && vresult.isValid()) {
            Pose3D robotPose = vresult.getBotpose_MT2();
            telemetry.addData("Target X", vresult.getTx());
            telemetry.addData("Target Y", vresult.getTy());
            telemetry.addData("Target Area", vresult.getTa());

        }
    }
}
