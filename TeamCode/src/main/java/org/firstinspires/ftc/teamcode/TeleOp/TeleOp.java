package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.impl.MotorEx;

    @com.qualcomm.robotcore.eventloop.opmode.TeleOp
    @Config
    public class TeleOp extends LinearOpMode {
        private DcMotor frontLeftMotor;
        private DcMotor frontRightMotor;
        private DcMotor backLeftMotor;
        private DcMotor backRightMotor;
        private DcMotor IntakeMotor;
        private ElapsedTime stopperDelayTimer = new ElapsedTime();

        private PIDFController PIDF;

        private DcMotorEx Turret;
        private long currTime3;

        public static double kp = 0.002;
        public static double ki = 0.0;
        public static double kd = 0.0;
        public static double kf = 0.0;
        private double lastError = 0;
        private double errorSum = 0;
        private long lastTime = 0;
        private InterpLUT graph;

        private DcMotorEx TopFlywheel;
        private DcMotorEx BottomFlywheel;

        public static int ticksPerSecond = 1500;
        public static int stopperThreshold = 80;
        private long currTime;
        private long currTime2;
        private long deltaTime;
        private Servo Stopper1;
        private Servo Stopper2;
        private long currTime4;
        private long rumbleTime;


        enum State {
            GENERAL_MOVEMENT,
            PEW_PEW

        }

        public double PID(double currentVelocity, double targetVelocity, long time) {
            double error = targetVelocity - currentVelocity;
            if (time <= 0) {
                time = 1;
            }
            double errorChange = (error - lastError) / time;
            errorSum += (error * time);
            lastError = error;
            return ((kp * error) + (ki * errorSum) + (kd * errorChange) + ((0.0007448464-(3.3333219e-7*targetVelocity)+(8.791839e-11*targetVelocity*targetVelocity)) * targetVelocity));//added new velocity thingy
        }


        State state;

        @Override
        public void runOpMode() throws InterruptedException {
            IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
            frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
            backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
            backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");
            frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            VoltageSensor Voltage = hardwareMap.voltageSensor.iterator().next();

            IMU imu = hardwareMap.get(IMU.class, "imu");

            TopFlywheel = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
            BottomFlywheel = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
            Turret = hardwareMap.get(DcMotorEx.class, "Turret");
            TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
            Stopper2 = hardwareMap.get(Servo.class, "Stopper2");

            IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            IntakeMotor.setPower(0);


            state = State.GENERAL_MOVEMENT;

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            //PIDF = new PIDFController(kp,ki,kd,kf);


            TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);


            waitForStart();
            lastTime = System.currentTimeMillis();


            while (opModeIsActive()) {

                double x = -gamepad1.left_stick_x * 1.1;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

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

                frontLeftMotor.setPower(frontLeft / -1);
                backLeftMotor.setPower(backLeft / -1);
                frontRightMotor.setPower(frontRight / 1);
                backRightMotor.setPower(backRight / 1);

                switch (state) {
                    case GENERAL_MOVEMENT:
                        currTime2 = 0;
                        currTime3 = 0;
                        gamepad1.rumble(500);
                        TopFlywheel.setPower(0);
                        BottomFlywheel.setPower(0);

                        if (gamepad1.left_trigger > 0.1) {
                            IntakeMotor.setPower(1);
                        } else if (gamepad1.right_trigger > 0.1) {
                            IntakeMotor.setPower(-1);
                        } else {
                            IntakeMotor.setPower(0);
                        }

                        if (gamepad1.dpad_left) {
                            gamepad2.rumble(1);
                            Turret.setPower(-0.5);
                        } else if (gamepad1.dpad_right) {
                            gamepad1.rumble(1);
                            Turret.setPower(0.5);
                        } else {
                            Turret.setPower(0);
                        }

                        Stopper1.setPosition(0.62);
                        Stopper2.setPosition(0.57);


                        if (gamepad1.right_bumper) {
                            stopperDelayTimer.reset();
                            state = State.PEW_PEW;

                        }


                        break;

                    case PEW_PEW:
                        gamepad1.rumble(500);
                        currTime = System.currentTimeMillis();
                        currTime2 = System.currentTimeMillis();
                        currTime4 = currTime3 - currTime2;
                        DcMotorEx Turret = new MotorEx("Turret").zeroed().getMotor();

                        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        deltaTime = currTime - lastTime;
                        double power1 = PID(TopFlywheel.getVelocity(), ticksPerSecond, deltaTime) * (12.0 / Voltage.getVoltage());
                        power1 = Math.max(-1.0, Math.min(1.0, power1));
                        lastTime = currTime;
                        TopFlywheel.setPower(-power1);
                        BottomFlywheel.setPower(-power1);

                        if (gamepad2.left_bumper){
                            Stopper1.setPosition(0.62);
                            Stopper2.setPosition(0.57);
                        } else if (gamepad2.right_bumper){
                            Stopper1.setPosition(0.77);
                            Stopper2.setPosition(0.77);
                        }
                        if (gamepad1.left_bumper) {
                            state = State.GENERAL_MOVEMENT;
                            Stopper1.setPosition(0.62);
                            Stopper2.setPosition(0.56);
                        }

                        IntakeMotor.setPower(-1);

                        if (Math.abs(TopFlywheel.getVelocity()-ticksPerSecond)<stopperThreshold) {
                            Stopper1.setPosition(0.77);
                            Stopper2.setPosition(0.7);
                        }

                        currTime3 = System.currentTimeMillis();
                        break;


                }

                telemetry.addData("State: ", state);
                telemetry.addData("Control Stick Left Y: ", -gamepad1.left_stick_y);
                telemetry.addData("Control Stick Left X: ", gamepad1.left_stick_x);
                telemetry.addData("Control Stick Right Y: ", -gamepad1.right_stick_y);
                telemetry.addData("Button A: ", gamepad1.a);
                telemetry.addData("Button B: ", gamepad1.b);
                telemetry.addData("Left Bumper: ", gamepad1.left_bumper);
                telemetry.addData("Right Bumper: ", gamepad1.right_bumper);
                telemetry.addData("Left Trigger: ", gamepad1.left_trigger);
                telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
                telemetry.addData("TopFlywheel Velocity", TopFlywheel.getVelocity());
                telemetry.addData("BottomFlywheel Velocity", BottomFlywheel.getVelocity());
                telemetry.addData("Target Speed", ticksPerSecond);
                telemetry.addData("Error", ticksPerSecond - TopFlywheel.getVelocity());
                telemetry.addData("Power", TopFlywheel.getPower());
                telemetry.addData("Current", TopFlywheel.getCurrent(CurrentUnit.AMPS) + BottomFlywheel.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Stopper1 Position: ", 0.77);
                telemetry.addData("Stopper2 Position: ", 0.7);
                telemetry.addData("Loading   *Shooting*", " ");
                telemetry.addData("               ↑    ", "");
                telemetry.addData("NIO NIO: ", currTime2);
                telemetry.update();

            }


        }

    }









