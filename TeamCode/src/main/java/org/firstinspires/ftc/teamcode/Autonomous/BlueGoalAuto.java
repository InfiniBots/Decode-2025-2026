package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathPoint;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Path sequence:
 * 1. Shoot 3 Preload -> (37, 107)
 * 2. First Ball Line -> (48, 84)
 * 3. Intake First Ball -> (20, 84)
 * 4. Shoot First Ball -> (37, 107)
 * 5. Second Ball Line -> (48, 60)
 * 6. Intake Second Ball -> (20, 60)
 * 7. Shoot Second Ball -> (37, 107) with curve
 * 8. Third Ball Line -> (48, 36)
 * 9. Intake Third Ball -> (16, 36)
 * 10. Shoot Third Ball -> (37, 107)
 */
@Autonomous
public class BlueGoalAuto extends NextFTCOpMode {

    public BlueGoalAuto() {
        addComponents(
                new SubsystemComponent(FlywheelVelocityPID.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static final Pose startPose = new Pose(34, 135.5, Math.toRadians(0));
    private static final Pose shootPose = new Pose(37, 107, Math.toRadians(180));

    private static final Pose firstBall = new Pose(48, 84, Math.toRadians(180));
    private static final Pose firstBallIntake = new Pose(20, 84, Math.toRadians(180));

    private static final Pose secondBall = new Pose(48, 60, Math.toRadians(180));
    private static final Pose secondBallIntake = new Pose(20, 60, Math.toRadians(180));

    private static final Pose thirdBall = new Pose(48, 36, Math.toRadians(180));
    private static final Pose thirdBallIntake = new Pose(16, 36, Math.toRadians(180));

    private PathChain shootPreload;
    private PathChain toFirstBall;
    private PathChain intakeFirstBall;
    private PathChain shootFirstBall;
    private PathChain toSecondBall;
    private PathChain intakeSecondBall;
    private PathChain shootSecondBall;
    private PathChain toThirdBall;
    private PathChain intakeThirdBall;
    private PathChain shootThirdBall;

    private static final double shootRpm = 3000.0;
    private static final double shootDelay = 0.5;
    private static final double intakeDelay = 0.3;
    private void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        (startPose),
                        (shootPose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toFirstBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (shootPose),
                        (firstBall)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        intakeFirstBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (firstBall),
                        (firstBallIntake)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        shootFirstBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (firstBallIntake),
                        (shootPose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        toSecondBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (shootPose),
                        (secondBall)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        intakeSecondBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (secondBall),
                        (secondBallIntake)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        shootSecondBall = follower.pathBuilder()
                .addPath(new BezierCurve(
                        (secondBallIntake),
                        (shootPose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        toThirdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (shootPose),
                        (thirdBall)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        intakeThirdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (thirdBall),
                        (thirdBallIntake)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        shootThirdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (thirdBallIntake),
                        (shootPose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private Command getAutonomousRoutine() {
        return new SequentialGroup(
        );
    }


    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);;
        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void onStartButtonPressed() {
        getAutonomousRoutine().schedule();
        opmodeTimer.resetTimer();
    }

    @Override
    public void onUpdate() {
        Pose currentPose = follower.getPose();
        follower.update();

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("X", "%.1f", currentPose.getX());
        telemetry.addData("Y", "%.1f", currentPose.getY());
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine();
        
        telemetry.addData("=== PATH ===", "");
        telemetry.addData("Following", follower.isBusy() ? "YES" : "NO");

        telemetry.update();
    }
}