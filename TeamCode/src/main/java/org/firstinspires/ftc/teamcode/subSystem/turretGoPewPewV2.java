package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class turretGoPewPewV2 {
    private DcMotorEx Turret;
    private DcMotorEx TurrMotor;
    private DcMotorEx TurrMotor2;
    private VoltageSensor Voltage;
    private Limelight3A limelight;

    private Servo shotAngler;
    public boolean isRed;
    public Telemetry telemetry;
    public double distance;
    public LLResult llResult;
    //pid for shooter
    public static double shooter_kp = 0.002;
    public static double shooter_ki = 0.0;
    public static double shooter_kd = 0.0;
   // public static double kf = 0.0;
    private double shooter_lastError = 0;

    private double shooter_errorSum = 0;
    private long shooter_lastTime=0;
    private int shooter_target;
    //pid for turret
    public static double turret_kp = 0.002;
    public static double turret_ki = 0.0;
    public static double turret_kd = 0.0;
    private double turret_lastError = 0;

    private double turret_errorSum = 0;
    private long turret_lastTime;
    private int turret_target;
    public static double ticksPRotation=360;
    public double ticksPerAng=(ticksPRotation/360.0);

    public turretGoPewPewV2(LinearOpMode op, boolean isRed, Telemetry telemetry){
        this.telemetry=telemetry;
        this.isRed=isRed;
        Turret = op.hardwareMap.get(DcMotorEx.class,"Turret");
        TurrMotor = op.hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        TurrMotor2 = op.hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
        Voltage = op.hardwareMap.voltageSensor.iterator().next();
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
       // shotAngler = op.hardwareMap.get(Servo.class,"shotAngler");


        TurrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double distanceAprilTag(double ta) {
        double scale = 30692.95; // value requires fine tuning will do later
        double distance = Math.sqrt(scale/ta) + 2;
        return distance;
    }
    public double shooter_PID(double currentVelocity, double targetVelocity, long time) {
        double error = targetVelocity - currentVelocity;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - shooter_lastError) / time;
        shooter_errorSum += (error * time);
        shooter_lastError = error;
        double power = ((shooter_kp * error) + (shooter_ki * shooter_errorSum) + (shooter_kd * errorChange) + ((0.0007448464-(3.3333219e-7*targetVelocity)+(8.791839e-11*targetVelocity*targetVelocity)) * targetVelocity))*(12.0/Voltage.getVoltage());//added new velocity thingy
        power=Math.max(-1.0, Math.min(1.0, power));
        return power;
    }
    public double turret_PID(double currPos, double targetPos, long time){
        double error = targetPos - currPos;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - turret_lastError) / time;
        turret_errorSum += (error * time);
        turret_lastError = error;
        return ((turret_kp*error)+(turret_ki*turret_errorSum)+(turret_kd*errorChange))*(12.0/Voltage.getVoltage());
    }
    public void shootingSpeed(){
        if (llResult != null&&llResult.isValid()) {
            distance = distanceAprilTag(llResult.getTa());
            telemetry.addData("limelight distance working",true);
        }
        shooter_target =(int)(6553.762-(167.7485*distance)+(2.001088*Math.pow(distance,2))-(0.01014018*Math.pow(distance,3))+(0.00001876297*Math.pow(distance,4)));
    }

    public void setSpeed(int targetSpeed){
        shooter_target =targetSpeed;
    }
    public void updateShooter(long time){
        llResult = limelight.getLatestResult();
        long delaTime=time - shooter_lastTime;
        double power= shooter_PID((TurrMotor.getVelocity()+TurrMotor2.getVelocity())/2, shooter_target,delaTime);
        shooter_lastTime = time;
        TurrMotor.setPower(-power);
        TurrMotor2.setPower(-power);
    }
    public void updateTurret(long time){
        llResult = limelight.getLatestResult();
        if (llResult != null&&llResult.isValid()) {
            double angle = distanceAprilTag(llResult.getTx());
            telemetry.addData("limelight angle working",true);
            turret_target=(int)(Turret.getCurrentPosition()+ticksPerAng*angle);
        }
        long deltaTime=time-turret_lastTime;
        turret_lastTime=time;
        double pow=turret_PID(Turret.getCurrentPosition(),turret_target,deltaTime);
        Turret.setPower(pow);
    }
    public boolean shooterIsAtSpeed(){
        if((Math.abs(TurrMotor.getVelocity()+TurrMotor2.getVelocity())/2- shooter_target)<67){
            shooter_errorSum =0;
            return true;
        }else{
            return false;
        }
    }

}

