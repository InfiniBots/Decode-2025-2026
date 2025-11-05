package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous
public class blueNearAuto extends LinearOpMode {
    private Robot robot;
    public Follower follower;
    public static boolean openGate=true;
    public String state = "start";
    private static final Pose start =new Pose(34, 135.3, Math.toRadians(180));
    private static final Pose shooting =  new Pose(39, 106.000, Math.toRadians(-139));

    private static final Pose ballStack_1 = new Pose(51, 84, Math.toRadians(180));
    private static final Pose intakingBalls_1 = new Pose(18, 84, Math.toRadians(180));
    private static final Pose openGateControl = new Pose(27, 80, Math.toRadians(0));

    private static final Pose intakingBalls_1_openGate = new Pose(18.000, 79.000, Math.toRadians(180));


    private static final Pose ballStack_2 = new Pose(51, 59, Math.toRadians(180));
    private static final Pose intakingBalls_2 = new Pose(10.5, 60, Math.toRadians(180));
    private static final Pose noTouchGate = new Pose(58,56);

    private static final Pose ballStack_3 = new Pose(51, 36, Math.toRadians(180));
    private static final Pose intakingBalls_3 = new Pose(10.5, 36, Math.toRadians(180));
    public static final Pose finalShoots = new Pose(43,120, Math.toRadians(-120));
    private static final Pose finalShootC1 =new Pose(60,36);
    private static final Pose finalShootC2 = new Pose(29,97);


    public long startShooting;
    public long startGate;
    public static int shootingSpeed=1450;
    public static int chillspeed=670;
    public long startIntaking;
    public static int intakingThreshold=670;
    public static int shootingThreshold=2000;
    public static int holdGateThreshold=2000;
    public static double intakeDelay = 0.3;
    public static double tTolerance=0.99;
    private PathChain Preload;
    private PathChain toBallStack_1;
    private PathChain toIntakingBalls_1;
    private PathChain toIntakingBalls_1_openGate;
    private PathChain shootBall_1;
    private PathChain toBallStack_2;
    private PathChain toIntakingBalls_2;
    private PathChain shootBall_2;
    private PathChain toBallStack_3;
    private PathChain toIntakingBalls_3;
    private PathChain shootBall_3;

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

        toIntakingBalls_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(ballStack_1, intakingBalls_1)
                )
                .setConstantHeadingInterpolation(intakingBalls_1.getHeading())
                .build();
        toIntakingBalls_1_openGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(intakingBalls_1,openGateControl,intakingBalls_1_openGate)
                )
                .setLinearHeadingInterpolation(ballStack_1.getHeading(),intakingBalls_1_openGate.getHeading())
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

        toIntakingBalls_2 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(ballStack_2, intakingBalls_2)
                )
                .setConstantHeadingInterpolation(intakingBalls_2.getHeading())
                .build();

        shootBall_2 =  follower.pathBuilder()
                .addPath(
                        new BezierCurve(intakingBalls_2,noTouchGate, shooting)
                )
                .setLinearHeadingInterpolation(intakingBalls_2.getHeading(), shooting.getHeading())
                .build();

        toBallStack_3 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(shooting, ballStack_3)
                )
                .setLinearHeadingInterpolation(shooting.getHeading(), ballStack_3.getHeading())
                .build();

        toIntakingBalls_3 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(ballStack_3, intakingBalls_3)
                )
                .setConstantHeadingInterpolation(intakingBalls_3.getHeading())
                .build();

        shootBall_3 =  follower.pathBuilder()
                .addPath(
                        new BezierCurve(intakingBalls_3,finalShootC1,finalShootC2 , finalShoots)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
    private boolean isAtPose() {
        return follower.getCurrentPath().getClosestPointTValue()>=tTolerance;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry=new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot=new Robot(this,telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();
        robot.Mode = "Driving";
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
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==67){
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
                        state = "intakingBalls_1";
                        follower.followPath(toIntakingBalls_1);
                    }
                    break;
                case "intakingBalls_1":
                    robot.intakingApproval = true;
                    if (!follower.isBusy()) {
                        if (openGate) {
                            if (robot.curTime - startIntaking >= (intakingThreshold)) {
                                robot.intakingApproval = false;
                                follower.followPath(toIntakingBalls_1_openGate);
                                startGate=robot.curTime;
                                state="gate";
                            }
                        } else {
                            if (robot.curTime - startIntaking >= intakingThreshold) {
                                robot.intakingApproval = false;
                                robot.chillShooterSpeed=shootingSpeed;
                                state = "shootBall_1";
                                follower.followPath(shootBall_1);
                            }
                        }
                    } else {
                        startIntaking = robot.curTime;
                    }
                    break;
                case "gate":
                    if(!follower.isBusy()) {
                        if (robot.curTime - startGate >= (holdGateThreshold)) {
                            robot.chillShooterSpeed = shootingSpeed;
                            state = "shootBall_1";
                            follower.followPath(shootBall_1);
                        }
                    }
                    break;
                case "shootBall_1":
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==67){
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
                        state = "intakingBalls_2";
                        follower.followPath(toIntakingBalls_2);
                    }
                    break;
                case "intakingBalls_2":
                    robot.intakingApproval=true;
                    if(!follower.isBusy()){
                        if(robot.curTime-startIntaking>=intakingThreshold){
                            robot.chillShooterSpeed=shootingSpeed;
                            state = "shootBall_2";
                            follower.followPath(shootBall_2);
                        }
                    }else{
                        startIntaking = robot.curTime;
                    }
                    break;
                case "shootBall_2":
                    if(follower.getPose().getY()<72){
                        robot.intakingApproval=false;
                    }
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==67){
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
                        state = "intakingBalls_3";
                        follower.followPath(toIntakingBalls_3);
                    }
                    break;
                case "intakingBalls_3":
                    robot.intakingApproval=true;
                    if(!follower.isBusy()){
                        if(robot.curTime-startIntaking>=intakingThreshold){
                            robot.chillShooterSpeed=shootingSpeed-100;
                            state = "shootBall_3";
                            follower.followPath(shootBall_3);

                        }
                    }else{
                        startIntaking = robot.curTime;
                    }
                    break;
                case "shootBall_3":
                    if(follower.getPose().getY()<48){
                        robot.intakingApproval=false;
                    }
                    if(!follower.isBusy()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold||robot.ballsLaunched==67){
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
            telemetry.addData("translational error",follower.getTranslationalError());
            telemetry.addData("drive error",follower.getDriveError());
            telemetry.addData("heading error",follower.getHeadingError());
            telemetry.update();
        }
    }
}
