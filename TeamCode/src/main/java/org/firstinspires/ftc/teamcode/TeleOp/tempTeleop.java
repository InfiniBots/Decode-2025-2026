package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subSystem.LimelightTracking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.ArrayList;

@TeleOp
@Config
public class tempTeleop extends LinearOpMode {
    public boolean isRed=true;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx IntakeMotor;
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    public LimelightTracking lltracking;
    public static double kp = 0.002;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0;
    private double lastError = 0;
    private double errorSum = 0;
    private long lastTime = 0;
    private DcMotorEx TopFlywheel;
    private DcMotorEx BottomFlywheel;
    public static int ticksPerSecond = 1500;
    public static int stopperThreshold = 80;
    private long currTime;
    private long deltaTime;
    private Servo Stopper1;
    private Servo Stopper2;
    public ArrayList<Double> cycles = new ArrayList<>();
    public long cycleTime;
    public double cycleAvg = 0;
    public static boolean usePedroMode = true;
    private Follower follower;
    private boolean prevRightStickButton = false;

    enum State {
        GENERAL_MOVEMENT,
        PEW_PEW

    }
    State state;
    public PathChain park;
    public boolean once=true;
    public void buildPath(){
        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), (isRed?(new Pose(38.7,33.3)):(new Pose(105.3,33.3))))
                )
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-90))
                .build();
    }

    public double PID(double currentVelocity, double targetVelocity, long time) {
        double error = targetVelocity - currentVelocity;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - lastError) / time;
        errorSum += (error * time);
        lastError = error;
        return ((kp * error) + (ki * errorSum) + (kd * errorChange) + ((0.0007448464 - (3.3333219e-7 * targetVelocity) + (8.791839e-11 * targetVelocity * targetVelocity)) * targetVelocity));//added new velocity thingy
    }
    public void updateShooter(){
        deltaTime = currTime - lastTime;
        double power1 = PID(Math.max(TopFlywheel.getVelocity(), BottomFlywheel.getVelocity()), ticksPerSecond, deltaTime) * (12.0 / Voltage.getVoltage());
        power1 = Math.max(-1.0, Math.min(1.0, power1));
        lastTime = currTime;
        TopFlywheel.setPower(-power1);
        BottomFlywheel.setPower(-power1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        lltracking = new LimelightTracking(this,telemetry);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Voltage = hardwareMap.voltageSensor.iterator().next();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        TopFlywheel = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        BottomFlywheel = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
        TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
        Stopper2 = hardwareMap.get(Servo.class, "Stopper2");

        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setPower(0);

        state = State.GENERAL_MOVEMENT;



        waitForStart();
        lastTime = System.currentTimeMillis();
        cycleTime = System.currentTimeMillis();
        follower.startTeleopDrive();
        follower.update();


        while (opModeIsActive()) {
            currTime = System.currentTimeMillis();
            updateShooter();
            follower.update();

            LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

            double totalCurrentAmps = controlHub.getCurrent(CurrentUnit.AMPS);

            LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

            double expansionCurrentAmps = expansionHub.getCurrent(CurrentUnit.AMPS);

            boolean rsb = gamepad1.right_stick_button;
            if (rsb && !prevRightStickButton) {
                usePedroMode = !usePedroMode;
                if (usePedroMode) {
                    follower.startTeleopDrive();
                } else {
                    follower.setTeleOpDrive(0, 0, 0, true);
                }
            }
            if(gamepad1.x){
                if(once) {
                    buildPath();
                    once=false;
                    follower.followPath(park);
                }
            }else{
                once=true;
            }
            prevRightStickButton = rsb;

            if (usePedroMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1,
                        -gamepad1.right_stick_x,
                        true
                );
                if(gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x==0){

                }
            } else {
                double x = -gamepad1.left_stick_x * 1.1;
                double y = -gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                double frontLeft = power * cos / max + turn;
                double frontRight = power * sin / max - turn;
                double backLeft = power * sin / max + turn;
                double backRight = power * cos / max - turn;

                if ((power + Math.abs(turn)) > 0.85) {
                    frontLeft /= power + Math.abs(turn);
                    frontRight /= power + Math.abs(turn);
                    backLeft /= power + Math.abs(turn);
                    backRight /= power + Math.abs(turn);
                }

                frontLeftMotor.setPower(frontLeft * 0.85);
                backLeftMotor.setPower(backLeft * 0.85);
                frontRightMotor.setPower(frontRight * 0.85);
                backRightMotor.setPower(backRight * 0.85);
            }
            switch (state) {
                case GENERAL_MOVEMENT:
                    ticksPerSecond=670;
                    if (gamepad1.left_trigger > 0.1) {
                        IntakeMotor.setPower(1);
                    } else if (gamepad1.right_trigger > 0.1) {
                        IntakeMotor.setPower(-1);
                    } else {
                        IntakeMotor.setPower(0);
                    }

                    if (gamepad1.dpad_left) {
                        Turret.setPower(-0.5);
                    } else if (gamepad1.dpad_right) {
                        Turret.setPower(0.5);
                    } else {
                        Turret.setPower(0);
                    }
                    Stopper1.setPosition(0.767);
                    Stopper2.setPosition(0.62);

                    if (gamepad1.right_bumper) {
                        state = State.PEW_PEW;

                    }


                    break;

                case PEW_PEW:
                    ticksPerSecond= lltracking.shootingSpeed()!=-4167?lltracking.shootingSpeed():ticksPerSecond;
                    if (gamepad2.a) {
                        Stopper1.setPosition(1);
                        Stopper2.setPosition(1);
                    }
                    if (gamepad1.left_bumper) {
                        state = State.GENERAL_MOVEMENT;
                        double cyc = (currTime - cycleTime) / 1000;
                        cycles.add(cyc);
                        cycleTime = currTime;
                    }

                    IntakeMotor.setPower(-1);

                    if (Math.abs(TopFlywheel.getVelocity() - ticksPerSecond) < stopperThreshold) {
                        Stopper1.setPosition(1);
                        Stopper2.setPosition(1);
                    }

                    break;


            }



            telemetry.addData("State: ", state);
            telemetry.addData("TopFlywheel Velocity", TopFlywheel.getVelocity());
            telemetry.addData("BottomFlywheel Velocity", BottomFlywheel.getVelocity());
            telemetry.addData("Target Speed", ticksPerSecond);
            telemetry.addData("Error", ticksPerSecond - TopFlywheel.getVelocity());
            telemetry.addData("Power", TopFlywheel.getPower());
            telemetry.addData("Current of shooter", TopFlywheel.getCurrent(CurrentUnit.AMPS) + BottomFlywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Drive Mode: ", usePedroMode ? "Pedro Pathing" : "Regular Mecanum");
            telemetry.addData("Pedro Pose: ", follower.getPose());
            telemetry.addData("Pedro Velocity: ", follower.getVelocity());
            telemetry.addData("Control Hub Current: ", totalCurrentAmps);
            telemetry.addData("Expansion Hub 2: ", expansionCurrentAmps);
            telemetry.addData("intake current", IntakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftRear", backLeftMotor.getPower());
            telemetry.update();

        }
        for (int i = 0; i < cycles.size(); i++) {
            telemetry.addData("Cycle " + (i + 1) + ": ", cycles.get(i));
            cycleAvg += cycles.get(i);
            telemetry.update();
        }
        telemetry.addData("Cycle avg: ", cycleAvg / cycles.size());
        telemetry.update();
    }
}