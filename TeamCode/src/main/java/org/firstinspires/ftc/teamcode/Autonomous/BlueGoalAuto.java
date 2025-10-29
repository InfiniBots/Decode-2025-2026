package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subSystem.VelocityPIDController;

import dev.nextftc.core.commands.groups.SequentialGroup;
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
@Config
@Autonomous
public class BlueGoalAuto extends NextFTCOpMode {

    private VelocityPIDController pidController;
    private DcMotorEx TopFlywheel;
    private DcMotorEx BottomFlywheel;
    private DcMotor IntakeMotor;
    private Servo Stopper1;
    private Servo Stopper2;
    public static double stopper1Close = 0.62;
    public static double stopper2Close = 0.57;
    public static double stopper1Open = 0.77;
    public static double stopper2Open = 0.7;
    private long lastTime = 0;
    public static double ticksPerSecond = 1500.0;
    public static double velocityThreshold = 60.0;
    private VoltageSensor voltageSensor;

    public BlueGoalAuto() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static final Pose startPose = new Pose(34, 135.5, Math.toRadians(180));
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


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(shootPreload, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    Intake();
                    updateFlywheel();
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if(isFlywheelAtSpeed() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    StopperOpen();
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toFirstBall, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onInit() {
        TopFlywheel = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        BottomFlywheel = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setPower(0);
        Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
        Stopper2 = hardwareMap.get(Servo.class, "Stopper2");
        TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);;
        pidController = new VelocityPIDController();
        StopperClose();
        buildPaths();
        follower.setStartingPose(startPose);
    }
    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        lastTime = System.currentTimeMillis();
        setPathState(0);
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

    private void updateFlywheel() {
        long currTime = System.currentTimeMillis();
        long deltaTime = currTime - lastTime;

        double currentVelocity = TopFlywheel.getVelocity();
        double power = pidController.PID(currentVelocity, ticksPerSecond, deltaTime) *(12.0/ voltageSensor.getVoltage());
        power = Math.max(-1.0, Math.min(1.0, power));

        TopFlywheel.setPower(-power);
        BottomFlywheel.setPower(-power);

        lastTime = currTime;
    }
    private boolean isFlywheelAtSpeed() {
        return Math.abs(TopFlywheel.getVelocity() - ticksPerSecond) < velocityThreshold;
    }

    private void stopFlywheel() {
        TopFlywheel.setPower(0);
        BottomFlywheel.setPower(0);
    }

    private void StopperOpen()  {
        Stopper1.setPosition(stopper1Open);
        Stopper2.setPosition(stopper2Open);
    }

    private void StopperClose()  {
        Stopper1.setPosition(stopper1Close);
        Stopper2.setPosition(stopper2Open);
    }

    private void Intake() {
        IntakeMotor.setPower(-1);
    }

    private void Outtake() {
        IntakeMotor.setPower(1);
    }
}
