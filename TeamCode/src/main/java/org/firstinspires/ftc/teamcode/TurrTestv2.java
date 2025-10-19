package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.control.Controller;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class TurrTestv2 extends LinearOpMode {
    private DcMotorEx TurrMotor;
    private DcMotorEx TurrMotor2;
    private InterpLUT graph;

    public static int ticksPerSecond = 700;
    private long currTime;
    private long deltaTime;

    //PID stuff
    //private PIDFController PIDF;
    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0;
    private double lastError = 0;
    private double errorSum = 0;
    private long lastTime = 0;

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

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //PIDF = new PIDFController(kp,ki,kd,kf);
        TurrMotor = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        TurrMotor2 = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");

        TurrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            currTime = System.currentTimeMillis();
            //PIDF.setPIDF(kp,ki,kd,kf);
           // double pidPow=PIDF.calculate(TurrMotor.getVelocity(),ticksPerSecond);
            if (gamepad1.a) {
                deltaTime = currTime - lastTime;
                double power = PID(TurrMotor.getVelocity(), ticksPerSecond, deltaTime);
                lastTime = currTime;
                TurrMotor.setPower(-power);
                TurrMotor2.setPower(-power);
            } else {
                TurrMotor.setPower(0);
                TurrMotor2.setPower(0);
                lastTime = currTime;
            }

            telemetry.addData("TurrMotor Velocity", TurrMotor.getVelocity());
            telemetry.addData("TurrMotor2 Velocity", TurrMotor2.getVelocity());
            telemetry.addData("Target Speed", ticksPerSecond);
            telemetry.addData("Error", ticksPerSecond - TurrMotor.getVelocity());
            telemetry.addData("Power", TurrMotor.getPower());
            telemetry.addData("Current", TurrMotor.getCurrent(CurrentUnit.AMPS) + TurrMotor2.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}