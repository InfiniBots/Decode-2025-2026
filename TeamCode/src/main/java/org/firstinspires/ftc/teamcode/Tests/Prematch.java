    package org.firstinspires.ftc.teamcode.Tests;

    import static org.firstinspires.ftc.teamcode.TeleOp.Cast_Ration.isRed;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import org.firstinspires.ftc.robotcore.external.Telemetry;

    @TeleOp
    @Config
    public class Prematch extends LinearOpMode {
    private DcMotorEx TurrMotor;
    private DcMotorEx TurrMotor2;
    private DcMotorEx TurretMotor;
    private DcMotorEx IntakeMotor;
    private Servo Stopper1;
    private Servo Stopper2;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    public static boolean flywheelCheck = false;
    public static boolean stopper1Open = false;
    private boolean lastY = false;
    public static boolean stopper2Open = false;
    public static boolean stopper2TestEnabled = false;
    private boolean lastBumperCombo = false;

    public static double  stopper1OpenPos  = 0.767;
    public static  double stopper1ClosePos = 1.0;
    public static double stopper2OpenPos  = 0.62;
    public static double stopper2ClosePos = 1.0;
    public static int tps = 1500;

        @Override
        public void runOpMode() throws InterruptedException {

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            TurrMotor = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
            TurrMotor2 = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
            TurrMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            TurrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TurrMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
            IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
            Stopper2 = hardwareMap.get(Servo.class, "Stopper2");
            Stopper1.setPosition(stopper1ClosePos);

            TurretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
            TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            waitForStart();

            while(opModeIsActive()){

                double x = gamepad1.left_stick_x * 1.1;
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
                    frontLeft  /= power + Math.abs(turn);
                    frontRight /= power + Math.abs(turn);
                    backLeft   /= power + Math.abs(turn);
                    backRight  /= power + Math.abs(turn);
                }

                frontLeftMotor.setPower(frontLeft);
                backLeftMotor.setPower(backLeft);
                frontRightMotor.setPower(frontRight);
                backRightMotor.setPower(backRight);

                if (gamepad1.left_trigger > 0.1) {
                    IntakeMotor.setPower(1.0);
                } else if (gamepad1.right_trigger > 0.1) {
                    IntakeMotor.setPower(-1.0);
                } else {
                    IntakeMotor.setPower(0.0);
                }

                if (gamepad1.dpad_left) {
                    TurretMotor.setPower(-0.4);
                } else if (gamepad1.dpad_right) {
                    TurretMotor.setPower(0.4);
                } else {
                    TurretMotor.setPower(0.0);
                }

                if (gamepad1.y && !lastY) {
                    flywheelCheck = !flywheelCheck;
                }
                lastY = gamepad1.y;
                if (flywheelCheck) {
                    TurrMotor.setVelocity(tps);
                    TurrMotor2.setVelocity(tps);
                } else {
                    TurrMotor.setPower(0);
                    TurrMotor2.setPower(0);
                }


                if (gamepad1.a) {
                    stopper1Open = true;
                } else if (gamepad1.x) {
                    stopper1Open = false;
                }
                Stopper1.setPosition(stopper1Open ? stopper1OpenPos : stopper1ClosePos);

                boolean bumperCombo = gamepad1.left_bumper && gamepad1.right_bumper;
                if (bumperCombo && !lastBumperCombo) {
                    stopper2TestEnabled = !stopper2TestEnabled;
                }
                lastBumperCombo = bumperCombo;
                if (stopper2TestEnabled) {
                    if (gamepad1.b) {
                        stopper2Open = true;
                    } else if (gamepad1.y) {
                        stopper2Open = false;
                    }
                    Stopper2.setPosition(stopper2Open ? stopper2OpenPos : stopper2ClosePos);
                } else {
                    stopper2Open = false;
                    Stopper2.setPosition(stopper2ClosePos);
                }

                telemetry.addData("Flywheels On", flywheelCheck);
                telemetry.addData("Ticks Per Second", tps);
                telemetry.addData("TopFlywheel Vel", TurrMotor.getVelocity());
                telemetry.addData("BottomFlywheel Vel", TurrMotor2.getVelocity());

                telemetry.addData("Turret Pos", TurretMotor.getCurrentPosition());
                telemetry.addData("Turret Power", TurretMotor.getPower());

                telemetry.addData("Stopper1 Open", stopper1Open);
                telemetry.addData("Stopper2 Open", stopper2Open);

                telemetry.addData("FL Power", frontLeftMotor.getPower());
                telemetry.addData("FR Power", frontRightMotor.getPower());
                telemetry.addData("BL Power", backLeftMotor.getPower());
                telemetry.addData("BR Power", backRightMotor.getPower());
                telemetry.addData("isRed", isRed);
                telemetry.update();

            }
        }
    }
