package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
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
    private Robot robot;
    public Follower follower;
    public Telemetry telemetry;
    public String state = "start";
    private static final Pose start = new Pose(110.000, 135.500, Math.toRadians(90));
    private static final Pose shooting = new Pose(107.000, 106.000, Math.toRadians(50));

    private static final Pose ballStack_1 = new Pose(96.000, 83.500, Math.toRadians(0));
    private static final Pose intakingBalls_1 = new Pose(125.000, 83.500, Math.toRadians(0));

    private static final Pose ballStack_2 = new Pose(96.000, 59.500, Math.toRadians(0));
    private static final Pose intakingBalls_2 = new Pose(125.000, 59.500, Math.toRadians(0));

    private static final Pose ballStack_3 = new Pose(96.000, 35.500, Math.toRadians(0));
    private static final Pose intakingBalls_3 = new Pose(125.000, 35.500, Math.toRadians(0));


    public long startShooting;
    public long startIntaking;
    public static int intakingThreshold=500;
    public static int shootingThreshold=2000;
    public static double intakeDelay = 0.3;
    public static double tTolerance=0.99;
    private PathChain Preload;
    private PathChain toBallStack_1;
    private PathChain toIntakingBalls_1;
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
                        new BezierLine(intakingBalls_2, shooting)
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
                        new BezierLine(intakingBalls_3, shooting)
                )
                .setLinearHeadingInterpolation(intakingBalls_3.getHeading(), shooting.getHeading())
                .build();
    }
    private boolean isAtPose() {
        return follower.getCurrentPath().getClosestPointTValue()>=tTolerance;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
                    follower.update();
                    telemetry.addData("auto started: ",true);
                    state="preload";
                    break;
                case "preload":
                    if(isAtPose()){
                        robot.Mode = "shooting";
                        if(robot.curTime-startShooting>=shootingThreshold){
                            robot.Mode = "Driving";
                            state = "toBallStack_1";
                            follower.followPath(toBallStack_1);
                        }
                    }else{
                        startShooting = robot.curTime;
                    }
                    break;
                    case "toBallStack_1":
                        if(isAtPose()){
                            state = "intakingBalls_1";
                            follower.followPath(toIntakingBalls_1);
                        }
                        break;
                        case "intakingBalls_1":
                            robot.intakingApproval=true;
                            if(isAtPose()){
                                if(robot.curTime-startIntaking>=intakingThreshold){
                                    robot.intakingApproval=false;
                                    state = "shootBall_1";
                                    follower.followPath(shootBall_1);
                                    }
                                }else{
                                startIntaking = robot.curTime;
                            }
                            break;
                            case "shootBall_1":
                                if(isAtPose()){
                                    robot.Mode = "shooting";
                                    if(robot.curTime-startShooting>=shootingThreshold){
                                        robot.Mode = "Driving";
                                        state = "toBallStack_2";
                                        follower.followPath(toBallStack_2);
                                    }
                                }else{
                                    startShooting = robot.curTime;
                                }
                                break;
                            case "toBallStack_2":
                                if(isAtPose()){
                                    state = "intakingBalls_2";
                                    follower.followPath(toIntakingBalls_2);
                                }
                                break;
                            case "intakingBalls_2":
                                robot.intakingApproval=true;
                                if(isAtPose()){
                                    if(robot.curTime-startIntaking>=intakingThreshold){
                                        robot.intakingApproval=false;
                                        state = "shootBall_2";
                                        follower.followPath(shootBall_2);
                                    }
                                }else{
                                    startIntaking = robot.curTime;
                                }
                                break;
                            case "shootBall_2":
                                if(isAtPose()){
                                    robot.Mode = "shooting";
                                    if(robot.curTime-startShooting>=shootingThreshold){
                                        robot.Mode = "Driving";
                                        state = "toBallStack_3";
                                        follower.followPath(toBallStack_3);
                                    }
                                }else{
                                    startShooting = robot.curTime;
                                }
                                break;
                            case "toBallStack_3":
                                if(isAtPose()){
                                    state = "intakingBalls_3";
                                    follower.followPath(toIntakingBalls_3);
                                }
                                break;
                            case "intakingBalls_3":
                                robot.intakingApproval=true;
                                if(isAtPose()){
                                    if(robot.curTime-startIntaking>=intakingThreshold){
                                        robot.intakingApproval=false;
                                        state = "shootBall_3";
                                        follower.followPath(shootBall_3);
            
                                    }
                                }else{
                                    startIntaking = robot.curTime;
                                }
                                break;
                            case "shootBall_3":
                                if(isAtPose()){
                                    robot.Mode = "shooting";
                                    if(robot.curTime-startShooting>=shootingThreshold){
                                        robot.Mode = "Driving";
                                    }
                                }else{
                                    startShooting = robot.curTime;
                                }
                                break;
                        }
            robot.UpdateRobot();
            follower.update();
            telemetry.update();
        }
    }
}
