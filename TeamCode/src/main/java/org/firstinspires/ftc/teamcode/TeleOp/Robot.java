package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.Cast_Ration.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystem.eatingBalls;
import org.firstinspires.ftc.teamcode.subSystem.turretGoPewPewV2;

import java.util.List;

@Config
public class Robot {
    public eatingBalls eatingBalls;
    public turretGoPewPewV2 turretGoPewPewV2;
    public Drive drive;
    public Telemetry telemetry;
    public List<LynxModule> allHubs;
    public long curTime;
    public String Mode="Driving";
    public static boolean issRED =isRed;//if we red or blue
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
        allHubs = op.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void UpdateRobot(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        curTime=System.currentTimeMillis();
        turretGoPewPewV2.updateShooter(curTime);
        turretGoPewPewV2.holdTurret();
        boolean shotIsAtSpeed = turretGoPewPewV2.shooterIsAtSpeed();
        double shootSped = turretGoPewPewV2.shooterGetSpeed();
       // turretGoPewPewV2.updateTurret(curTime);
        //turret aiming code goes outside of switch case want it always on
        switch (Mode){
            case "Driving":
                turretGoPewPewV2.setSpeed(chillShooterSpeed);
                lastSpeed = shootSped;
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
                if(shotIsAtSpeed||continuous){
                    continuous=true;//so it doesnt stop after every shot since each shot decreases vel speed
                    eatingBalls.intakeOpen();
                    eatingBalls.intaking();

                }
                if(continuous) {
                    double currentSpeed = shotIsAtSpeed?turretGoPewPewV2.shooter_target:shootSped;
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
                }

                telemetry.addData("State: ",Mode);
                telemetry.addData("Continuous: ",continuous);
                break;
        }
        telemetry.addData("shooter speed",shootSped);
        telemetry.addData("isShooterAtSpeed",shotIsAtSpeed);

       // telemetry.addData("shooter is at pos: ", turretGoPewPewV2.shooterIsAtSpeed());
    }
}
