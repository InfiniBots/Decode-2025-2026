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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subSystem.VelocityPIDController;

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
public class BackupRedGoalAuto extends OpMode {

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

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static final Pose startPose = new Pose(110, 135.5, Math.toRadians(0));
    private static final Pose shootPose = new Pose(107, 107, Math.toRadians(325));

    private static final Pose firstBall = new Pose(96, 84, Math.toRadians(0));
    private static final Pose firstBallIntake = new Pose(124, 84, Math.toRadians(0));

    private static final Pose secondBall = new Pose(96, 60, Math.toRadians(0));
    private static final Pose secondBallIntake = new Pose(124, 60, Math.toRadians(0));

    private static final Pose thirdBall = new Pose(96, 36, Math.toRadians(0));
    private static final Pose thirdBallIntake = new Pose(124, 36, Math.toRadians(0));

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

    private void buildPaths() {
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        (startPose),
                        (shootPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toFirstBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (shootPose),
                        (firstBall)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstBall.getHeading())
                .build();


        intakeFirstBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (firstBall),
                        (firstBallIntake)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        shootFirstBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (firstBallIntake),
                        (shootPose)
                ))
                .setLinearHeadingInterpolation(firstBallIntake.getHeading(), shootPose.getHeading())
                .build();


        toSecondBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (shootPose),
                        (secondBall)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondBall.getHeading())
                .build();


        intakeSecondBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (secondBall),
                        (secondBallIntake)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        shootSecondBall = follower.pathBuilder()
                .addPath(new BezierCurve(
                        (secondBallIntake),
                        (shootPose)
                ))
                .setLinearHeadingInterpolation(secondBallIntake.getHeading(), shootPose.getHeading())
                .build();


        toThirdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (shootPose),
                        (thirdBall)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdBall.getHeading())
                .build();


        intakeThirdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (thirdBall),
                        (thirdBallIntake)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        shootThirdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        (thirdBallIntake),
                        (shootPose)
                ))
                .setLinearHeadingInterpolation(thirdBallIntake.getHeading(), shootPose.getHeading())
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
                    IntakeStop();
                    StopperClose();
                    stopFlywheel();
                    follower.followPath(toFirstBall, true);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    Intake();
                    follower.followPath(intakeFirstBall, true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    IntakeStop();
                    follower.followPath(shootFirstBall, true);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    Intake();
                    updateFlywheel();
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if(isFlywheelAtSpeed() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    StopperOpen();
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    IntakeStop();
                    StopperClose();
                    stopFlywheel();
                    follower.followPath(toSecondBall, true);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    Intake();
                    follower.followPath(intakeSecondBall, true);
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    IntakeStop();
                    follower.followPath(shootSecondBall, true);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    Intake();
                    updateFlywheel();
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if(isFlywheelAtSpeed() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    StopperOpen();
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    IntakeStop();
                    StopperClose();
                    stopFlywheel();
                    follower.followPath(toThirdBall, true);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    Intake();
                    follower.followPath(intakeThirdBall, true);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    IntakeStop();
                    follower.followPath(shootThirdBall, true);
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    Intake();
                    updateFlywheel();
                    pathTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if(isFlywheelAtSpeed() || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    StopperOpen();
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    IntakeStop();
                    StopperClose();
                    stopFlywheel();
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
    public void init() {
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
        follower = Constants.createFollower(hardwareMap);
        pidController = new VelocityPIDController();
        StopperClose();
        buildPaths();
        follower.setStartingPose(startPose);
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        lastTime = System.currentTimeMillis();
        setPathState(0);
    }

    @Override
    public void loop() {
        Pose currentPose = follower.getPose();
        follower.update();
        autonomousPathUpdate();

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
        Stopper2.setPosition(stopper2Close);
    }

    private void Intake() {
        IntakeMotor.setPower(-1);
    }

    private void IntakeStop() {
        IntakeMotor.setPower(0);
    }
    private void Outtake() {
        IntakeMotor.setPower(1);
    }
}
