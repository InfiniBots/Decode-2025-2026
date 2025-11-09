package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystem.eatingBalls;
import org.firstinspires.ftc.teamcode.subSystem.turretGoPewPewV2;

@Config
public class Robot {
    public eatingBalls eatingBalls;
    public turretGoPewPewV2 turretGoPewPewV2;
    public Drive drive;
    public Telemetry telemetry;
    public long curTime;
    public String Mode="Driving";
    public static boolean issRED =false;//if we red or blue
    public int chillShooterSpeed=670;
    public int setTargetSpeed=1500;
    public int ballsLaunched=0;
    private boolean continuous=false;
    public boolean intakingApproval=false;
    public double lastSpeed;
    public boolean lowering=false;
    public Robot(LinearOpMode op, Telemetry telemetry){
        this.telemetry = telemetry;
        drive = new Drive(op);
        eatingBalls = new eatingBalls(op, telemetry);
        turretGoPewPewV2 = new turretGoPewPewV2(op, issRED,telemetry);
    }
    public void UpdateRobot(){
        curTime=System.currentTimeMillis();
        turretGoPewPewV2.updateShooter(curTime);
        turretGoPewPewV2.holdTurret();
       // turretGoPewPewV2.updateTurret(curTime);
        //turret aiming code goes outside of switch case want it always on
        switch (Mode){
            case "Driving":
                turretGoPewPewV2.setSpeed(chillShooterSpeed);
                lastSpeed = turretGoPewPewV2.shooterGetSpeed();
                ballsLaunched=0;
                eatingBalls.intakeClose();
                if(intakingApproval){
                    eatingBalls.intaking();
                }else {
                    eatingBalls.chilling();
                }
                continuous=false;
                break;
            case "shooting":
                turretGoPewPewV2.setSpeed(setTargetSpeed);
                //turretGoPewPewV2.shootingSpeed();
                if(turretGoPewPewV2.shooterIsAtSpeed()||continuous){
                    continuous=true;//so it doesnt stop after every shot since each shot decreases vel speed
                    eatingBalls.intakeOpen();
                    eatingBalls.intaking();

                }
                if(continuous) {
                    double currentSpeed = turretGoPewPewV2.shooterIsAtSpeed()?turretGoPewPewV2.shooter_target:turretGoPewPewV2.shooterGetSpeed();
                    if (currentSpeed < lastSpeed) {
                        lastSpeed = currentSpeed;
                        lowering = true;
                        telemetry.addData("in1","inside");
                    }
                    else if (lowering && currentSpeed > lastSpeed) {
                        lastSpeed = currentSpeed;
                        ballsLaunched++;
                        lowering = false;
                        telemetry.addData("in2","inside2");
                    }else{
                        lastSpeed = currentSpeed;
                    }
                    telemetry.addData("currentSpeed",currentSpeed);
                }
                telemetry.addData("lastspeed",lastSpeed);

                telemetry.addData("State: ",Mode);
                telemetry.addData("Continuous: ",continuous);
                break;
        }
        telemetry.addData("turrspeed",turretGoPewPewV2.shooterGetSpeed());
        telemetry.addData("isturratSpeed",turretGoPewPewV2.shooterIsAtSpeed());

       // telemetry.addData("shooter is at pos: ", turretGoPewPewV2.shooterIsAtSpeed());
    }
}
