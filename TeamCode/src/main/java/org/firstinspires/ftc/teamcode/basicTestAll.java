package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TurrTestv2.kd;
import static org.firstinspires.ftc.teamcode.TurrTestv2.kf;
import static org.firstinspires.ftc.teamcode.TurrTestv2.ki;
import static org.firstinspires.ftc.teamcode.TurrTestv2.kp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.field.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Drive;

@Config
@TeleOp
public class basicTestAll extends LinearOpMode {
    private Drive drive;
    public Servo Stopper1;
    public Servo Stopper2;
    private DcMotorEx TurrMotor;
    private DcMotorEx intakeStage1;
    private DcMotorEx TurrMotor2;
    private VoltageSensor Voltage;
    public static double intakePow=-1;
    public static double ticksPSec=1500;
    public static double chillTickPSec=0;
    public static double intakeShootPow=-1;
    private long currTime;
    private long deltaTime;
    private double power;
    private double lastError = 0;

    private double errorSum = 0;
    private long lastTime = 0;
    public static boolean shooterOn=false;
    public static boolean intakeOn=false;
    public void closeStop(){
        Stopper1.setPosition(0.62);
        Stopper2.setPosition(0.56);
    }
    public void openStop(){
        Stopper1.setPosition(0.77);
        Stopper2.setPosition(0.7);
    }
    public double PID(double currentVelocity, double targetVelocity, long time) {
        double error = targetVelocity - currentVelocity;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - lastError) / time;
        errorSum += (error * time);
        lastError = error;
        return ((kp * error) + (ki * errorSum) + (kd * errorChange) + (kf * targetVelocity));//added new velocity thingy
        //kp and other values get from TurrTestv2
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(this);
        Stopper1=hardwareMap.get(Servo.class,"Stopper1");
        Stopper2=hardwareMap.get(Servo.class,"Stopper2");
        TurrMotor2 = hardwareMap.get(DcMotorEx.class,"BottomFlywheel");
        TurrMotor = hardwareMap.get(DcMotorEx.class,"TopFlywheel");
        intakeStage1 = hardwareMap.get(DcMotorEx.class,"Intake");
        Voltage=hardwareMap.voltageSensor.iterator().next();
        TurrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeStage1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lastTime=System.currentTimeMillis();
        waitForStart();
        closeStop();
        while(opModeIsActive()){
            currTime = System.currentTimeMillis();
            drive.driveInputs(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            if(gamepad1.right_trigger>0.5||intakeOn){
                closeStop();
                intakeStage1.setPower(intakePow);
                telemetry.addData("Status: ","intaking");
            }else if(!(gamepad1.a||shooterOn)){
                intakeStage1.setPower(0);
            }
            if (gamepad1.a||shooterOn) {
                deltaTime = currTime - lastTime;
                power = PID(TurrMotor.getVelocity(), ticksPSec, deltaTime)*(12.0/Voltage.getVoltage());
                lastTime = currTime;
                power=Math.max(-1.0, Math.min(1.0, power));
                TurrMotor.setPower(-power);
                TurrMotor2.setPower(-power);
                telemetry.addData("Status: ","gaining speed");
                if(Math.abs(TurrMotor.getVelocity()-ticksPSec)<100) {
                    openStop();
                    intakeStage1.setPower(intakeShootPow);
                    telemetry.addData("Status: ","pew pew");
                }
            } else {
                closeStop();
                deltaTime = currTime - lastTime;
                power = PID(TurrMotor.getVelocity(), chillTickPSec, deltaTime)*(12.0/Voltage.getVoltage());
                lastTime = currTime;
                power=Math.max(-1.0, Math.min(1.0, power));
                TurrMotor.setPower(-power);
                TurrMotor2.setPower(-power);
            }
            telemetry.addData("TurrMotor vel: ",TurrMotor.getVelocity());
            telemetry.update();
        }
    }
}
