package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
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

    private Servo shotAngler;
    public boolean isRed;
    public Telemetry telemetry;
    //pid
    public static double kp = 0.0009;
    public static double ki = 0.0000001;
    public static double kd = 0.09;
    public static double kf = 0.00055;
    private double lastError = 0;

    private double errorSum = 0;
    private long lastTime;
    private int target;

    public turretGoPewPewV2(LinearOpMode op, boolean isRed, Telemetry telemetry){
        this.telemetry=telemetry;
        this.isRed=isRed;
        Turret = op.hardwareMap.get(DcMotorEx.class,"Turret");
        TurrMotor = op.hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        TurrMotor2 = op.hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
        Voltage = op.hardwareMap.voltageSensor.iterator().next();
       // shotAngler = op.hardwareMap.get(Servo.class,"shotAngler");


        TurrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurrMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double PID(double currentVelocity, double targetVelocity, long time) {
        double error = targetVelocity - currentVelocity;
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - lastError) / time;
        errorSum += (error * time);
        lastError = error;
        double power=((kp * error) + (ki * errorSum) + (kd * errorChange) + (kf * targetVelocity))*(12.0/Voltage.getVoltage());
        power=Math.max(-1.0, Math.min(1.0, power));
        return power;
    }
    public void shootingSpeed(){
        //calc speed we do that later

    }
    public void setSpeed(int targetSpeed){
        target=targetSpeed;
        errorSum=0;
    }
    public void updateShooter(long time){
        long delaTime=time-lastTime;
        double power=PID((TurrMotor.getVelocity()+TurrMotor2.getVelocity())/2,target,delaTime);
        lastTime = time;
        TurrMotor.setPower(-power);
        TurrMotor2.setPower(-power);
    }
    public boolean shooterIsAtSpeed(){
        if((Math.abs(TurrMotor.getVelocity()+TurrMotor2.getVelocity())/2-target)<67){
            return true;
        }else{
            return false;
        }
    }

}

