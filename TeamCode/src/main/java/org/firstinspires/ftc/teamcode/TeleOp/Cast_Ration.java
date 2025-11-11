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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subSystem.LimelightTracking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.ArrayList;

@TeleOp
@Config
public class Cast_Ration extends LinearOpMode {
    public static boolean isRed =true;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx IntakeMotor;
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    public LimelightTracking lltracking;
    public static double kp = 0.003;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0;
    private double lastError = 0;
    private double errorSum = 0;
    private long lastTime = 0;
    private DcMotorEx TopFlywheel;
    private DcMotorEx BottomFlywheel;
    public static int ticksPerSecond = 0;
    public static int stopperThreshold = 80;
    public static boolean tracking=true;
    private long currTime;
    private long deltaTime;
    private Servo Stopper1;
    private Servo Stopper2;
    public ArrayList<Double> cycles = new ArrayList<>();
    public long cycleTime;
    public double cycleAvg = 0;
    public boolean turretzeroing=false;
    public static boolean usePedroMode = false;
    private Follower follower;
    private boolean prevRightStickButton = false;

    enum State {
        GENERAL_MOVEMENT,
        PEW_PEW

    }
    State state;
    public boolean once=true;
    public boolean turrToggle=true;
    public boolean turretOnOff =true;
    /*public PathChain park;
    public void buildPath(){
        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), (isRed ?(new Pose(38.7,33.3)):(new Pose(105.3,33.3))))
                )
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-90))
                .build();
    }*/

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
        follower.setPose(new Pose(80,80,Math.toRadians(-90)));
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
        //Stopper2 = hardwareMap.get(Servo.class, "Stopper2");

        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setPower(-0.1);

        state = State.GENERAL_MOVEMENT;



        waitForStart();
        lastTime = System.currentTimeMillis();
        cycleTime = System.currentTimeMillis();
        follower.startTeleopDrive();
        follower.update();


        while (opModeIsActive()) {
            currTime = System.currentTimeMillis();
            follower.update();

            LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

            double totalCurrentAmps = controlHub.getCurrent(CurrentUnit.AMPS);

            LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

            double expansionCurrentAmps = expansionHub.getCurrent(CurrentUnit.AMPS);
            if(gamepad2.a&& turretOnOff){
                tracking=!tracking;
                turretOnOff =false;
            }else if(!gamepad2.a){
                turretOnOff =true;
            }

            /*if(gamepad1.x&&Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.right_stick_x)+Math.abs(gamepad1.left_stick_y)==0){
                if(once) {
                    buildPath();
                    once=false;
                    follower.followPath(park);
                }

            }else if (!gamepad1.x) {
                once = true;*/


                double x = gamepad1.left_stick_x * 1.1;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x * 0.85;

                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                double frontLeft = power * cos / max + turn;
                double frontRight = power * sin / max - turn;
                double backLeft = power * sin / max + turn;
                double backRight = power * cos / max - turn;

                if ((power + Math.abs(turn)) > 1) {
                    frontLeft /= power + Math.abs(turn);
                    frontRight /= power + Math.abs(turn);
                    backLeft /= power + Math.abs(turn);
                    backRight /= power + Math.abs(turn);
                }

                frontLeftMotor.setPower(frontLeft);
                backLeftMotor.setPower(backLeft);
                frontRightMotor.setPower(frontRight);
                backRightMotor.setPower(backRight);

            switch (state) {
                case GENERAL_MOVEMENT:
                    if (gamepad1.left_trigger > 0.1) {
                        IntakeMotor.setPower(1);
                    } else if (gamepad1.right_trigger > 0.1) {
                        IntakeMotor.setPower(-1);
                    } else {
                        IntakeMotor.setPower(-0.2);
                    }
                    lltracking.turrReset();
                    /*if(gamepad1.a&&turrToggle){
                        lltracking.turrReset();
                        turretzeroing=!turretzeroing;
                        turrToggle=false;
                    }else if(!gamepad1.a){
                        turrToggle=true;
                    }
                    if(turretzeroing){
                        lltracking.turrReset();
                    }else{
                       if(tracking) lltracking.updateTurret(follower.getHeading(),follower.getPose().getX(), follower.getPose().getY(), gamepad1.right_stick_x);

                    }
                    telemetry.addData("turretzeroing",turretzeroing);
                    telemetry.addData("turrToggle",turrToggle);*/

                    if (gamepad2.dpad_left) {
                        Turret.setPower(-0.5);
                        tracking=false;
                    } else if (gamepad2.dpad_right) {
                        Turret.setPower(0.5);
                        tracking=false;
                    } else if(!tracking){
                        Turret.setPower(0);
                    }

                    if (gamepad1.right_bumper) {
                        state = State.PEW_PEW;

                    }

                    if (gamepad2.x) {
                        ticksPerSecond = 1500;
                    }else{
                    ticksPerSecond=0;
                    }

                    if (gamepad2.dpad_up) {
                        Stopper1.setPosition(1);
                        //Stopper2.setPosition(1);
                    } else if (gamepad2.dpad_down) {
                        Stopper1.setPosition(0);
                        //Stopper2.setPosition(0);
                    } else {
                        Stopper1.setPosition(0.767);
                        //Stopper2.setPosition(0.62);
                    }

                    break;

                case PEW_PEW:
                    if(tracking)lltracking.updateTurret(follower.getHeading(),follower.getPose().getX(), follower.getPose().getY(), gamepad1.right_stick_x);
                    //ticksPerSecond = lltracking.shootingSpeed()!=-4167?lltracking.shootingSpeed()-20:1500;
                    ticksPerSecond = 1500;
                    if (gamepad2.a) {
                        Stopper1.setPosition(1);
                       // Stopper2.setPosition(1);
                    }
                    if (gamepad1.left_bumper) {
                        state = State.GENERAL_MOVEMENT;
                        double cyc = (currTime - cycleTime) / 1000;
                        cycles.add(cyc);
                        cycleTime = currTime;
                        ticksPerSecond = 0;
                    }


                    IntakeMotor.setPower(-1);

                    if (gamepad2.dpad_up) {
                        Stopper1.setPosition(1);
                      //  Stopper2.setPosition(1);
                    } else if (gamepad2.dpad_down) {
                        Stopper1.setPosition(0);
                       // Stopper2.setPosition(0);
                    } else {
                        if (Math.abs(TopFlywheel.getVelocity() - ticksPerSecond) < stopperThreshold) {
                            Stopper1.setPosition(1);
                           // Stopper2.setPosition(1);
                        }
                    }


                    break;


            }


            updateShooter();
            telemetry.addData("State: ", state);
            telemetry.addData("TopFlywheel Velocity", TopFlywheel.getVelocity());
            telemetry.addData("BottomFlywheel Velocity", BottomFlywheel.getVelocity());
            telemetry.addData("Target Speed", ticksPerSecond);
            telemetry.addData("Error", ticksPerSecond - TopFlywheel.getVelocity());
            telemetry.addData("Power", TopFlywheel.getPower());
            telemetry.addData("Current of shooter", TopFlywheel.getCurrent(CurrentUnit.AMPS) + BottomFlywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Control Hub Current: ", totalCurrentAmps);
            telemetry.addData("Expansion Hub 2: ", expansionCurrentAmps);
            telemetry.addData("intake current", IntakeMotor.getCurrent(CurrentUnit.AMPS));
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