package org.firstinspires.ftc.teamcode.subSystem;

import android.graphics.Color;

import com.bylazar.field.Line;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;

public class turretGoPewPew  {
    private DcMotorEx turrMover;
    private DcMotorEx mainFlyWheel;
    private Servo shotAngler;

    //TODO tune like everything
    public boolean isRed;
    public static double ygoal=138; //prob decently accurate, got from pedro visualizer
    public double goalx;
    public static double speedBoost=0;//extra power
    public static int ticksFor1Rotation=10000;
    public static double slope=1;

    //pid
    public double ErrorSum=0;
    public double lastError=0;
    public double kp=0;
    public double ki=0;
    public double kd=0;
    public long prevTime=0;

    public turretGoPewPew(LinearOpMode op, boolean isRed){
        this.isRed=isRed;
        turrMover = op.hardwareMap.get(DcMotorEx.class,"turrMover");
        mainFlyWheel = op.hardwareMap.get(DcMotorEx.class,"mainFlyWheel");
        shotAngler = op.hardwareMap.get(Servo.class,"shotAngler");


        turrMover.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //angler
    private double idealAngle(double boty, double xdistance){
         double ydelta=ygoal-boty;
        double angOfElev=Math.atan(ydelta/xdistance);
        double angle=(Math.toRadians(90)+angOfElev)/2;
        return Math.toDegrees(angle);
    }
    public void angler(double boty, double xdistance){
        double angle=idealAngle(boty,xdistance);
        //TODO somehow convert angle to servo pos prob by graphing points os servos poses and their angle to get line of best fit
        double pos=angle/slope;//slope is what i would get from graphing points for example it could be 30deg/0.1 pos and if my angle is 60 and
        // divide by slope i get servos pos 0.2
        shotAngler.setPosition(pos);
    }
    //launcher
    private double idealpPow(double boty, double xdistance){
        double angle=Math.toRadians(idealAngle(boty,xdistance));
        double ydelta=ygoal-boty;
        double Vyt=(Math.tan(angle))*(xdistance);
        double xcomp=Math.sqrt(1/((ydelta-Vyt)/(0.5*(-9.8)*xdistance*xdistance)));
        double V=xcomp/Math.cos(angle);
        double maxSpeed=22.619467;//in m/s of our flywheel motor assuming it has 72mm diameter and 6k rpm like sushi said
        double power=V/maxSpeed;
        power=power+speedBoost;
        if(V>=maxSpeed){
            power=1;
        }
        return power;
    }
    public void launchPow(double boty, double xdistance){
        double pow=idealpPow(boty,xdistance);
        mainFlyWheel.setPower(pow);
    }
    //aimer
    private double PID(int initPos,long targetPos,long time){
        long Error = targetPos-initPos;
        if(time==0){
            time=1;
        }
        double errorChange=(Error-lastError)/time;
        ErrorSum+=(Error*time);
        lastError=Error;
        return ((kp*Error)+(ki*ErrorSum)+(kd*errorChange));
    }
    private long targetpos(double botx, double boty, double botTheta){
        goalx=isRed?136:11;
        double ticksPerAng=(ticksFor1Rotation/360);
        double xLength=(goalx-botx);
        double yLength=(ygoal-boty);
        double rawturrAngle=Math.toDegrees(Math.atan2(xLength,yLength));
        System.out.println("Turrangle with no robot heading: "+rawturrAngle);
        double turrAngle=rawturrAngle-botTheta;
        System.out.println("angle of turr: "+turrAngle);
        long ticks=Math.round(ticksPerAng*turrAngle);
        return ticks;
    }
    public void Aimer(double botx, double boty, double botTheta, long time){
        long Time=time-prevTime;
        prevTime=time;
        double pow=PID(turrMover.getCurrentPosition(),targetpos(botx, boty, botTheta),time);
        turrMover.setPower(pow);
    }

}
