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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.impl.MotorEx;

    @com.qualcomm.robotcore.eventloop.opmode.TeleOp
    @Config
    public class TeleOp extends LinearOpMode {
        private DcMotor leftFront;
        private DcMotor rightFront;
        private DcMotor leftRear;
        private DcMotor rightRear;
        private PIDFController PIDF;

        public static double kp = 0.0;
        public static double ki = 0.0;
        public static double kd = 0.0;
        public static double kf = 0.0;
        private double lastError = 0;
        private double errorSum = 0;
        private long lastTime = 0;
        private InterpLUT graph;

        private DcMotorEx TopFlywheel;
        private DcMotorEx BottomFlywheel;

        public static int ticksPerSecond = 700;
        private long currTime;
        private long deltaTime;
        private Servo Stopper1;
        private Servo Stopper2;


        enum State {
            GENERAL_MOVEMENT,
            PEW_PEW

        }

        private double PID(double currentVelocity, double targetVelocity, long time) {
            double error = targetVelocity - currentVelocity;
            if (time <= 0) {
                time = 1;
            }
            double errorChange = (error - lastError) / time;
            errorSum += (error * time);
            lastError = error;
            return ((kp * error) + (ki * errorSum) + (kd * errorChange) + (kf * targetVelocity));//added new velocity thingy
        }

        void getVel() {// we doing in ft and tickspersecond
            graph.add(  1, 1);//need to add points (ts is random right now)
            graph.createLUT();
        }

        State state;

        @Override
        public void runOpMode() throws InterruptedException {
            DcMotorEx intakeMotor = new MotorEx("Intake").zeroed().getMotor();
            leftFront = hardwareMap.dcMotor.get("leftFront");
            rightFront = hardwareMap.dcMotor.get("rightFront");
            leftRear = hardwareMap.dcMotor.get("leftRear");
            rightRear = hardwareMap.dcMotor.get("rightRear");
            VoltageSensor Voltage = hardwareMap.voltageSensor.iterator().next();

            IMU imu = hardwareMap.get(IMU.class, "imu");

            TopFlywheel = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
            BottomFlywheel = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
            TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
            Stopper2 = hardwareMap.get(Servo.class, "Stopper2");

            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setPower(0);


            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

            state = State.GENERAL_MOVEMENT;

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            //PIDF = new PIDFController(kp,ki,kd,kf);


            TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);


            waitForStart();
            lastTime = System.currentTimeMillis();


            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                leftFront.setPower((y + x + rx) / denominator);
                rightFront.setPower((y - x + rx) / denominator);
                leftRear.setPower((y - x - rx) / denominator);
                rightRear.setPower((y + x - rx) / denominator);



                switch (state) {
                    case GENERAL_MOVEMENT:

                        intakeMotor.setPower(gamepad1.right_trigger); // negative cuz iirc y up is neg down is pos for wtv rzn

                        if (gamepad1.right_bumper) {
                            state = State.PEW_PEW;
                        }

                        Stopper1.setPosition(0.62);
                        Stopper2.setPosition(0.56);

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
                        telemetry.addData("Power: ", -gamepad1.right_stick_y);
                        telemetry.addData("Stopper1 Position: ", 0.62);
                        telemetry.addData("Stopper2 Position: ", 0.57);
                        telemetry.update();
                        break;

                    case PEW_PEW:
                        currTime = System.currentTimeMillis();
                        if (gamepad1.a) {
                            deltaTime = currTime - lastTime;
                            double power = PID(TopFlywheel.getVelocity(), ticksPerSecond, deltaTime)*(12.0/Voltage.getVoltage());
                            power=Math.max(-1.0, Math.min(1.0, power));
                            lastTime = currTime;
                            TopFlywheel.setPower(-power);
                            BottomFlywheel.setPower(-power);
                        } else {
                            TopFlywheel.setPower(0);
                            BottomFlywheel.setPower(0);
                            lastTime = currTime;
                        }
                        if (gamepad1.left_bumper) {
                            state = State.GENERAL_MOVEMENT;
                        }

                        Stopper1.setPosition(0.77);
                        Stopper2.setPosition(0.7);


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
                        telemetry.update();
                        break;


                }

            }


        }
    }









