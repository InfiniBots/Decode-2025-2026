package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.TeleOp.Robot.isRed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous
public class redGoalAuto extends LinearOpMode {
    private boolean issRed=isRed;
    private Robot robot;
    public Follower follower;
    public static boolean openGate=true;
    public String state = "start";
    private static final Pose start = new Pose(110.000, 135.500, Math.toRadians(0));
    private static final Pose shooting = new Pose(107.000, 106.000, Math.toRadians(-45));

    private static final Pose ballStack_1 = new Pose(96.000, 83.000, Math.toRadians(0));
    private static final Pose intakingBalls_1 = new Pose(122.500, 83.000, Math.toRadians(0));
    private static final Pose openGateControl = new Pose(102, 81, Math.toRadians(0));

    private static final Pose intakingBalls_1_openGate = new Pose(131, 75, Math.toRadians(-90));
    private static final Pose turnOffIntake1 = new Pose(121.00, 81);
    private static final Pose ballStack_2 = new Pose(96.000, 59.000, Math.toRadians(0));
    private static final Pose intakingBalls_2 = new Pose(131.000, 59.000, Math.toRadians(0));
    private static final Pose noTouchGate = new Pose(98,50);
    private static final Pose turnOffIntake2 = new Pose(120.00, 59);
    private static final Pose ballStack_3 = new Pose(94.000, 39.000, Math.toRadians(0));
    private static final Pose intakingBalls_3 = new Pose(131.000, 39.000, Math.toRadians(0));
    public static final Pose finalShoot = new Pose(94,117, Math.toRadians(-44));
    private static final Pose finalShootC1 =new Pose(101.5,39.5);
    private static final Pose finalShootC2 = new Pose(100,106);
    private static final Pose turnOffIntake3 = new Pose(117, 47);


    public long startShooting;
    public long startGate;
    public static int shootingSpeed=1420;
    public static int chillspeed=670;
    public long startIntaking;
    public static int intakingThreshold=670;
    public static int shootingThreshold=3000;
    public static int holdGateThreshold=1500;
    private PathChain Preload;
    private PathChain toBallStack_1;
    private PathChain combinedIntakePath_1;
    private PathChain shootBall_1;
    private PathChain toBallStack_2;
    private PathChain combinedIntakePath_2;
    private PathChain toBallStack_3;
    private PathChain combinedIntakePath_3;
    //public static int turrPose=650;
    public static double maxThresh=0.67;

    public void buildPaths(){
        Preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, shooting)
                )
                .setLinearHeadingInterpolation(start.getHeading(), shooting.getHeading())
                .build();
        toBallStack_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(shooting, ballStack_1)
                )
                .setLinearHeadingInterpolation(shooting.getHeading(), ballStack_1.getHeading())
                .build();

        combinedIntakePath_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(ballStack_1, intakingBalls_1)
                )
                .setConstantHeadingInterpolation(intakingBalls_1.getHeading())
                .addPath(
                        new BezierCurve(intakingBalls_1,openGateControl,intakingBalls_1_openGate)
                )
                .setLinearHeadingInterpolation(intakingBalls_1.getHeading(),intakingBalls_1_openGate.getHeading())
                .addPoseCallback(turnOffIntake1, () -> {
                    robot.intakingApproval = false;
                }, 0.7)
                .build();

        shootBall_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(intakingBalls_1, shooting)
                )
                .setLinearHeadingInterpolation(intakingBalls_1.getHeading(), shooting.getHeading())
                .build();

        toBallStack_2 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(shooting, ballStack_2)
                )
                .setLinearHeadingInterpolation(shooting.getHeading(), ballStack_2.getHeading())
                .build();

        combinedIntakePath_2 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(ballStack_2, intakingBalls_2)
                )
                .setConstantHeadingInterpolation(intakingBalls_2.getHeading())
                .addPath(
                        new BezierCurve(intakingBalls_2,noTouchGate, shooting)
                )
                .setLinearHeadingInterpolation(intakingBalls_2.getHeading(), shooting.getHeading())
                .addPoseCallback(turnOffIntake2,()->{
                    robot.intakingApproval=false;
                    robot.chillShooterSpeed=shootingSpeed;
                    follower.setMaxPower(1);
                },0.6)
                .build();

        toBallStack_3 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(shooting, ballStack_3)
                )
                .setLinearHeadingInterpolation(shooting.getHeading(), ballStack_3.getHeading())
                .build();

        combinedIntakePath_3 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(ballStack_3, intakingBalls_3)
                )
                .setConstantHeadingInterpolation(intakingBalls_3.getHeading())
                .addPath(
                        new BezierCurve(intakingBalls_3,finalShootC1,finalShootC2 ,finalShoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPoseCallback(turnOffIntake3,()->{
                    robot.intakingApproval=false;
                    robot.chillShooterSpeed=shootingSpeed+40;
                    follower.setMaxPower(1);
                },0.6)
                .build();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry=new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot=new Robot(this,telemetry);
        /*if(!issRed){
            start=start.mirror();
            shooting=shooting.mirror();
            ballStack_1=ballStack_1.mirror();
            ballStack_2=ballStack_2.mirror();
            ballStack_3=ballStack_3.mirror();
            intakingBalls_1=intakingBalls_1.mirror();
            intakingBalls_2=intakingBalls_2.mirror();
            intakingBalls_3=intakingBalls_3.mirror();
            noTouchGate=noTouchGate.mirror();
            intakingBalls_1_openGate=intakingBalls_1_openGate.mirror();
            openGateControl=openGateControl.mirror();
            turnOffIntake1=turnOffIntake1.mirror();
            turnOffIntake2=turnOffIntake2.mirror();
            turnOffIntake3=turnOffIntake3.mirror();
            finalShoot=finalShoot.mirror();
            finalShootC1=finalShootC1.mirror();
            finalShootC2=finalShootC2.mirror();
        }*/
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();
        robot.Mode = "Driving";
        // robot.turretGoPewPewV2.turretSetPose(turrPose);
        waitForStart();
        while (opModeIsActive()){
            follower.update();
            switch (state){
                case "start":
                    follower.followPath(Preload);
                    robot.setTargetSpeed=shootingSpeed;
                    robot.chillShooterSpeed=shootingSpeed;
                    follower.update();
                    telemetry.addData("auto started: ",true);
                    state="preload";
                    break;
                case "preload":
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==3){
                            robot.Mode = "Driving";
                            state = "toBallStack_1";
                            follower.followPath(toBallStack_1);
                        }
                    }else{
                        startShooting = robot.curTime;
                    }
                    break;
                case "toBallStack_1":
                    if(!follower.isBusy()){
                        state = "comboIntake1";
                        robot.intakingApproval = true;
                        follower.followPath(combinedIntakePath_1);
                        follower.setMaxPower(maxThresh);
                    }
                    break;
                case "comboIntake1":
                    if(!follower.isBusy()) {
                        if(robot.curTime-startGate>=holdGateThreshold) {
                            robot.chillShooterSpeed = shootingSpeed;
                            state = "shootBall_1";
                            follower.followPath(shootBall_1);
                            follower.setMaxPower(1);
                        }
                    }else{
                        startGate = robot.curTime;
                    }
                    break;
                case "shootBall_1":
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==3){
                            robot.chillShooterSpeed=chillspeed;
                            robot.Mode = "Driving";
                            state = "toBallStack_2";
                            follower.followPath(toBallStack_2);
                        }
                    }else{
                        startShooting = robot.curTime;
                    }
                    break;
                case "toBallStack_2":
                    if(!follower.isBusy()){
                        robot.intakingApproval=true;
                        state = "comboIntake2";
                        follower.followPath(combinedIntakePath_2);
                        follower.setMaxPower(maxThresh);
                    }
                    break;
                case "comboIntake2":
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==3){
                            robot.chillShooterSpeed=chillspeed;
                            robot.Mode = "Driving";
                            state = "toBallStack_3";
                            follower.followPath(toBallStack_3);
                        }
                    }else{
                        startShooting = robot.curTime;
                    }
                    break;
                case "toBallStack_3":
                    if(!follower.isBusy()){
                        robot.intakingApproval=true;
                        state = "comboIntake3";
                        follower.followPath(combinedIntakePath_3);
                        follower.setMaxPower(maxThresh);
                    }
                    break;
                case "comboIntake3":
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==3){
                            robot.chillShooterSpeed=chillspeed;
                            robot.Mode = "Driving";
                            robot.chillShooterSpeed=0;
                        }
                    }else{
                        startShooting = robot.curTime;
                    }
                    break;
            }
            robot.UpdateRobot();
            follower.update();
            telemetry.addData("ballslaunched",robot.ballsLaunched);
            telemetry.addData("pose",follower.getPose());
            telemetry.update();
        }
    }
}
