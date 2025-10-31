package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.hardware.impl.MotorEx;
@TeleOp
@Config
public class turretMover extends LinearOpMode {
    private DcMotorEx Turret;
    private Limelight3A limelight;
    private LLResult llResult;
    private VoltageSensor Voltage;
    private GoBildaPinpointDriver pinpoint;
    private long curTime;
    private long deltaTime;
    private long lastTime;
    public static boolean turrOn=false;

    public static double turret_kp = 0.02;
    public static double turret_ki = 0.00000006;
    public static double turret_kd = 0.003;
    private double turret_lastError = 0;
    private double power;

    private double turret_errorSum = 0;
    private long turret_lastTime;
    public static int turret_target=0;
    public static double ticksPRotation = 2288;
    public double ticksPerAng = (ticksPRotation / 360.0);
    public void configurePinpoint(){
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public double distanceAprilTag(double ta) {
        double scale = 30692.95; // value requires fine tuning will do later
        double distance = Math.sqrt(scale / ta) + 2;
        return distance;
    }

    public void updateTurret(long time) {
        llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double angle = distanceAprilTag(llResult.getTx());
            telemetry.addData("limelight angle working", true);
            turret_target = (int) (Turret.getCurrentPosition() + ticksPerAng * angle);
        }
        long deltaTime = time - turret_lastTime;
        turret_lastTime = time;
        double pow = turret_PID(Turret.getCurrentPosition(), turret_target, deltaTime);
        Turret.setPower(pow);
    }

    public double turret_PID(double currPos, double targetPos, long time) {
        double error = targetPos - currPos;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - turret_lastError) / time;
        turret_errorSum += (error * time);
        turret_lastError = error;
        telemetry.addData("inside","inside pid");
        return ((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)) * (12.0 / Voltage.getVoltage());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Voltage = hardwareMap.voltageSensor.iterator().next();
        limelight.pipelineSwitch(1);
        lastTime = System.currentTimeMillis();
        waitForStart();
        while (opModeIsActive()) {
            curTime = System.currentTimeMillis();
            if (gamepad1.x||turrOn) {
                deltaTime = curTime - lastTime;
                power = turret_PID(Turret.getCurrentPosition(), turret_target, deltaTime) * (12.0 / Voltage.getVoltage());
                telemetry.addData("power",power);
                power = Math.max(-1.0, Math.min(1.0, power));
                lastTime = curTime;
                Turret.setPower(power);
            } else {
                Turret.setPower(0);
                lastTime = curTime;

            }
            telemetry.addData("Turr pos",Turret.getCurrentPosition());
            telemetry.addData("target",turret_target);
            telemetry.addData("power recieved",Turret.getPower());
            telemetry.addData("Voltage",Voltage.getVoltage());
            telemetry.update();
        }
    }
}
