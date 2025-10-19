package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.impl.MotorEx;

    @com.qualcomm.robotcore.eventloop.opmode.TeleOp
    @Config
    public class TeleOp extends LinearOpMode {
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor rearLeft;
        private DcMotor rearRight;
        private boolean readyForIntake;

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

        void getVel(){// we doing in ft and tickspersecond
            graph.add(1,1);//need to add points (ts is random right now)
            graph.createLUT();
        }

        State state;

        @Override
        public void runOpMode() throws InterruptedException {
            DcMotorEx intakeMotor = new MotorEx("Intake").zeroed().getMotor();
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            rearLeft = hardwareMap.dcMotor.get("rearLeft");
            rearRight = hardwareMap.dcMotor.get("rearRight");

            TopFlywheel = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
            BottomFlywheel = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
            TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setPower(0);



            rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            state = State.GENERAL_MOVEMENT;

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            //PIDF = new PIDFController(kp,ki,kd,kf);


            TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);




            waitForStart();
            lastTime = System.currentTimeMillis();


            while (opModeIsActive()){
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.left_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                frontLeft.setPower((y + x + rx) / denominator);
                frontRight.setPower((y - x + rx) / denominator);
                rearLeft.setPower((y - x - rx) / denominator);
                rearRight.setPower((y + x - rx) / denominator);

                telemetry.addData("State: ", state);
                telemetry.addData("Control Stick Left Y: ", -gamepad1.left_stick_y);
                telemetry.addData("Control Stick Left X: ", gamepad1.left_stick_x);
                telemetry.addData("Control Stick Right Y: ", -gamepad1.right_stick_y);
                telemetry.addData("Button A: ", gamepad1.a);
                telemetry.addData("Button B: ", gamepad1.b);
                telemetry.addData("Left Bumper: ", gamepad1.left_bumper);
                telemetry.addData("Right Bumper: ", gamepad1.right_bumper);
                telemetry.update();

                switch (state) {
                    case GENERAL_MOVEMENT:

                        if (gamepad1.a) {
                            readyForIntake = true;
                            if (-gamepad1.right_stick_y >= 0) { // this exists to ensure there isnt a negative value for the motor.
                                intakeMotor.setPower(-gamepad1.right_stick_y); // negative cuz iirc y up is neg down is pos for wtv rzn
                            }
                        } else if (gamepad1.b) {
                            readyForIntake = false;
                            intakeMotor.setPower(0);
                        }
                        if (gamepad1.right_bumper){
                            state = State.PEW_PEW;
                        }
                        telemetry.addData("Intake: ", readyForIntake);
                        telemetry.addData("Power: ", -gamepad1.right_stick_y);
                        telemetry.update();
                        break;

                    case PEW_PEW:
                        currTime = System.currentTimeMillis();
                        if (gamepad1.a) {
                            deltaTime = currTime - lastTime;
                            double power = PID(TopFlywheel.getVelocity(), ticksPerSecond, deltaTime);
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

                        telemetry.addData("TurrMotor Velocity", TopFlywheel.getVelocity());
                        telemetry.addData("TurrMotor2 Velocity", BottomFlywheel.getVelocity());
                        telemetry.addData("Target Speed", ticksPerSecond);
                        telemetry.addData("Error", ticksPerSecond - TopFlywheel.getVelocity());
                        telemetry.addData("Power", TopFlywheel.getPower());
                        telemetry.addData("Current", TopFlywheel.getCurrent(CurrentUnit.AMPS) + BottomFlywheel.getCurrent(CurrentUnit.AMPS));
                        telemetry.update();


                }

                }


                }




            }





