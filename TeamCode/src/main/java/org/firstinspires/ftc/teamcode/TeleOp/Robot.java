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
    public static boolean isRed=false;//if we red or blue
    public int chillShooterSpeed=670;
    public int setTargetSpeed=1500;
    public int ballsLaunched=0;
    public boolean checking;
    private boolean continuous=false;
    public boolean intakingApproval=false;
    public Robot(LinearOpMode op, Telemetry telemetry){
        this.telemetry = telemetry;
        drive = new Drive(op);
        eatingBalls = new eatingBalls(op, telemetry);
        turretGoPewPewV2 = new turretGoPewPewV2(op,isRed,telemetry);
    }
    public void UpdateRobot(){
        curTime=System.currentTimeMillis();
        turretGoPewPewV2.updateShooter(curTime);
       // turretGoPewPewV2.updateTurret(curTime);
        //turret aiming code goes outside of switch case want it always on
        switch (Mode){
            case "Driving":
                turretGoPewPewV2.setSpeed(chillShooterSpeed);
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
                    if(turretGoPewPewV2.shooterIsAtSpeed()){
                        checking=true;
                    }
                    if(checking&&turretGoPewPewV2.shooterGetSpeed()<turretGoPewPewV2.shooter_target-200){
                        ballsLaunched++;
                        checking=false;
                    }
                }
        }
        telemetry.addData("State: ",Mode);
        telemetry.addData("Continuous: ",continuous);
       // telemetry.addData("shooter is at pos: ", turretGoPewPewV2.shooterIsAtSpeed());
    }
}
