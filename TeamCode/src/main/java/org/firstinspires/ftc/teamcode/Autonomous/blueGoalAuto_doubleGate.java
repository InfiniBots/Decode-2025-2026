package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.StorePos.curPose;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.ballStack_1;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.ballStack_2;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.finalShootC1;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.finalShootC2;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.intakingBalls_1;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.intakingBalls_2;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.intakingBalls_3;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.noTouchGate;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.openGate1Control;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.openGate2Control;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.start;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.turnOffIntake1;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.turnOffIntake2;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto_doubleGate.turnOffIntake3;
import static org.firstinspires.ftc.teamcode.TeleOp.Robot.issRED;

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
public class blueGoalAuto_doubleGate extends LinearOpMode {
    private boolean issRed= issRED;
    private Robot robot;
    public Follower follower;
    public String state = "start";
    public static final  Pose b_start = start.mirror();
    public static final  Pose b_shooting = new Pose(38, 108.000, Math.toRadians(-135));

    public static final  Pose b_ballStack_1 = ballStack_1.mirror();
    public static final  Pose b_intakingBalls_1 = intakingBalls_1.mirror();
    public static final  Pose b_openGate1Control =  openGate1Control.mirror();

    public static final  Pose b_intakingBalls_1_openGate = new Pose(13, 79, Math.toRadians(90));
    public static final  Pose b_turnOffIntake1 = turnOffIntake1.mirror();
    public static final  Pose b_ballStack_2 = ballStack_2.mirror();
    public static final  Pose b_intakingBalls_2 = intakingBalls_2.mirror();
    public static final  Pose b_openGate2Control = new Pose(25, 63, Math.toRadians(0));
    public static final  Pose b_intakingBalls_2_openGate =  new Pose(13, 74, Math.toRadians(-90));
    public static final  Pose b_turnOffIntake2 = turnOffIntake2.mirror();
    public static final  Pose b_ballStack_3 =  new Pose(49, 38.000, Math.toRadians(180));
    public static final  Pose b_intakingBalls_3 = intakingBalls_3.mirror();
    public static final Pose b_finalShoot =  new Pose(53,120, Math.toRadians(-117));
    public static final  Pose b_finalShootC1 =finalShootC1.mirror();
    public static final  Pose b_finalShootC2 = finalShootC2.mirror();
    public static final  Pose b_turnOffIntake3 = turnOffIntake3.mirror();



    public long startShooting;
    public long oscilDelay;
    public static int oscilThresh=200;
    public long startGate;
    public static int shootingSpeed=1450;
    public static int chillspeed=670;
    public long startIntaking;
    public static int intakingThreshold=670;
    public static int shootingThreshold=2700;
    public static int holdGateThreshold=1500;
    private PathChain Preload;
    private PathChain toBallStack_1;
    private PathChain combinedIntakePath_1;
    private PathChain shootBall_2;
    private PathChain shootBall_1;
    private PathChain toBallStack_2;
    private PathChain combinedIntakePath_2;
    private PathChain toBallStack_3;
    private PathChain combinedIntakePath_3;
    //public static int turrPose=650;
    public static double maxThresh=0.8;

    public void buildPaths(){
        Preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(b_start, b_shooting)
                )
                .setLinearHeadingInterpolation(b_start.getHeading(), b_shooting.getHeading())
                .build();
        toBallStack_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_shooting, b_ballStack_1)
                )
                .setLinearHeadingInterpolation(b_shooting.getHeading(), b_ballStack_1.getHeading())
                .build();

        combinedIntakePath_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_ballStack_1, b_intakingBalls_1)
                )
                .setConstantHeadingInterpolation(b_intakingBalls_1.getHeading())
                .addPath(
                        new BezierCurve(b_intakingBalls_1, b_openGate1Control, b_intakingBalls_1_openGate)
                )
                .setLinearHeadingInterpolation(b_intakingBalls_1.getHeading(), b_intakingBalls_1_openGate.getHeading())
                .addPoseCallback(b_turnOffIntake1, () -> {
                    robot.intakingApproval = false;
                }, 0.7)
                .build();

        shootBall_1 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_intakingBalls_1, b_shooting)
                )
                .setLinearHeadingInterpolation(b_intakingBalls_1.getHeading(), b_shooting.getHeading())
                .build();

        toBallStack_2 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_shooting, b_ballStack_2)
                )
                .setLinearHeadingInterpolation(b_shooting.getHeading(), b_ballStack_2.getHeading())
                .build();

        combinedIntakePath_2 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_ballStack_2, b_intakingBalls_2)
                )
                .setConstantHeadingInterpolation(b_intakingBalls_2.getHeading())
                .addPath(
                        new BezierCurve(b_intakingBalls_2,b_openGate2Control,b_intakingBalls_2_openGate)
                )
                .setLinearHeadingInterpolation(b_intakingBalls_2.getHeading(), b_intakingBalls_2_openGate.getHeading())
                .addPoseCallback(b_turnOffIntake2,()->{
                    robot.intakingApproval=false;
                    robot.chillShooterSpeed=shootingSpeed;
                    follower.setMaxPower(1);
                },0.6)
                .build();
        shootBall_2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(b_intakingBalls_2_openGate,b_shooting)
                )
                .setLinearHeadingInterpolation(b_intakingBalls_2_openGate.getHeading(),b_shooting.getHeading())
                .build();

        toBallStack_3 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_shooting, b_ballStack_3)
                )
                .setLinearHeadingInterpolation(b_shooting.getHeading(), b_ballStack_3.getHeading())
                .build();

        combinedIntakePath_3 =  follower.pathBuilder()
                .addPath(
                        new BezierLine(b_ballStack_3, b_intakingBalls_3)
                )
                .setConstantHeadingInterpolation(b_intakingBalls_3.getHeading())
                .addPath(
                        new BezierCurve(b_intakingBalls_3, b_finalShootC1, b_finalShootC2, b_finalShoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPoseCallback(b_turnOffIntake3,()->{
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
        shootingSpeed=robot.turretGoPewPewV2.Voltage.getVoltage()>14.0?1450 :1467;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(b_start);
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
                        if(robot.curTime-oscilDelay>=oscilThresh) {
                            robot.Mode = "shooting";
                            if (robot.curTime - startShooting >= shootingThreshold || robot.ballsLaunched == 3) {
                                robot.chillShooterSpeed = chillspeed;
                                robot.Mode = "Driving";
                                state = "toBallStack_2";
                                follower.followPath(toBallStack_2);
                            }
                        }
                    }else{
                        startShooting = robot.curTime;
                        oscilDelay = robot.curTime;
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
                    if(!follower.isBusy()) {
                        if(robot.curTime-startGate>=holdGateThreshold-1000) {
                            robot.chillShooterSpeed = shootingSpeed;
                            state = "shootBall_2";
                            follower.followPath(shootBall_2);
                            follower.setMaxPower(1);
                        }
                    }else{
                        startGate = robot.curTime;
                    }
                    break;
                case "shootBall_2":
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
                        if(robot.curTime-oscilDelay>=oscilThresh) {
                            robot.Mode = "shooting";
                            if (robot.curTime - startShooting >= shootingThreshold || robot.ballsLaunched == 3) {
                                robot.chillShooterSpeed = chillspeed;
                                robot.Mode = "Driving";
                                robot.chillShooterSpeed = 0;
                            }
                        }
                    }else{
                        startShooting = robot.curTime;
                        oscilDelay=robot.curTime;
                    }
                    break;
            }
            curPose=follower.getPose();
            robot.UpdateRobot();
            follower.update();
            telemetry.addData("ballslaunched",robot.ballsLaunched);
            telemetry.addData("shooter act speed","");
            telemetry.addData("shooter speed","");
            telemetry.addData("pose",follower.getPose());
            telemetry.update();
        }
    }
}
