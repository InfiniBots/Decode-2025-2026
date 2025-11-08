package org.firstinspires.ftc.teamcode.subSystem;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LimelightTracking{
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    private double targetPosition = 0;
    private boolean isntGettingRecognized = false;
    private Limelight3A limelight;
    private long lastTime;
    private long curTime;
    public static double turret_kp = 0.02;
    public static double turret_ki = 0.00000006;
    public static double turret_kd = 0.003;
    private double turret_lastError = 0;
    private double turret_errorSum = 0;
    public static double ticksPRotation = 2288;
    public double ticksPerAng=1;
    private final double wishingX = 0.00;
    public LLResult result;
    public double error=0;
    public static double limit = 500;
    public boolean limiting=false;
    public double power=0;
    public double x=0;
    public double conversionRate = (564/90);
    public double distance=1;
    Telemetry telemetry;
    public LimelightTracking(LinearOpMode op, Telemetry telemetry){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret = op.hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);
        lastTime = System.currentTimeMillis();
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        Voltage = op.hardwareMap.voltageSensor.iterator().next();
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public double turret_PID(double currPos, double targetPos,long curtime) {
        long dtime=curtime-lastTime;
        lastTime=curtime;
        double error =targetPos-currPos;
        if (dtime <= 0) {
            dtime = 1;
        }
        double errorChange = (error - turret_lastError) / dtime;
        if(!limiting) {turret_errorSum += (error * dtime);}
        turret_lastError = error;
        return ((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)) * (12.0 / Voltage.getVoltage());
    }
    public void manualTurret(double manualTurretPower){
        Turret.setPower(manualTurretPower);
    }

    public void updateTurret(){
        curTime=System.currentTimeMillis();
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()){
            isntGettingRecognized = true; // TEMPORARY
        } else {
            x = result.getTx();
            ticksPerAng=(x / 360.0) * (384.5 * (140/16));
            targetPosition = Turret.getCurrentPosition()-x;
            if (x != 0.00){
                power = turret_PID(Turret.getCurrentPosition(),targetPosition,curTime);
                if (Turret.getCurrentPosition()>limit&&power>0){
                    power=0;
                    limiting=true;
                } else if (Turret.getCurrentPosition()<-limit&&power<0){
                    power=0;
                    limiting=true;
                }else{
                    limiting=false;
                }
                Turret.setPower(power);
            }

            telemetry.addData("isn'tGettingRecognized: ", isntGettingRecognized);
            telemetry.addData("target x: ", x);
            telemetry.addData("error: ", error);
        }
        telemetry.addData("limiting",limiting);
        telemetry.addData("isn'tGettingRecognized: ", isntGettingRecognized);
        telemetry.addData("currpos",Turret.getCurrentPosition());
        telemetry.update();

    }
                                                                                                
    public int shootingSpeed(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null&&llResult.isValid()) {
            distance = distanceAprilTag(llResult.getTa());
            telemetry.addData("limelight distance working (cm)",distance);
            return (int)(6553.762-(167.7485*distance)+(2.001088*Math.pow(distance,2))-(0.01014018*Math.pow(distance,3))+(0.00001876297*Math.pow(distance,4)));
        }else{
            return -4167;
        }
    }

    public void disableTurret(){
        Turret.setPower(0);
        telemetry.addData("Disabled !", " ");
    }
    private double distanceAprilTag(double ta) {
        double scale = 30692.95;
        double distance = Math.sqrt(scale/ta) + 2;
        return distance;
    }



}


