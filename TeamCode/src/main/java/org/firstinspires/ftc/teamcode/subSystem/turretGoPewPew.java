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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;

public class turretGoPewPew  {
    private DcMotorEx turrMover;
    private DcMotorEx mainFlyWheel;
    private Servo shotAngler;
    private RevColorSensorV3 colorSensor;
    public Telemetry telemetry;

    //TODO tune like everything
    public boolean isRed;
    public static double ygoal=138*0.0254; //prob decently accurate, got from pedro visualizer
    public static double goalHeight=1.1176; //in meters
    public static double turrHeight=0.4318; //in meters
    public double goalx;
    public static double speedBoost=0;//extra power
    public static int ticksFor1Rotation=10000;
    public static double slope=1;
    public static double delay=0.2;
    public static double ticksPerRev=28;

    //pid
    public double ErrorSum=0;
    public double lastError=0;
    public double kp=0;
    public double ki=0;
    public double kd=0;
    public long prevTime=0;
    //color stuff
    public static double fullThreshold=10;
    public static double greenMin=100;
    public static double greenMax=150;
    public static double purpleMin=270;
    public static double purpleMax=330;
    public float[] hsv=new float[3];

    public turretGoPewPew(LinearOpMode op, boolean isRed, Telemetry telemetry){
        this.telemetry=telemetry;
        this.isRed=isRed;
        goalx=isRed?136*0.0254:11*0.0254;
        turrMover = op.hardwareMap.get(DcMotorEx.class,"turrMover");
        mainFlyWheel = op.hardwareMap.get(DcMotorEx.class,"mainFlyWheel");
        shotAngler = op.hardwareMap.get(Servo.class,"shotAngler");
        colorSensor = op.hardwareMap.get(RevColorSensorV3.class,"colorSensor");


        turrMover.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //extras
    private double getxdistance(double boty, double botx){
        double boty_conv=boty*0.0254;//convert into meter
        double botx_conv=botx*0.0254;
        double botdeltax=goalx-botx_conv;
        double botdeltay=ygoal-boty_conv;
        double xdistance=Math.sqrt(botdeltay*botdeltay+botdeltax*botdeltax);
        return xdistance;
    }
    //indexing calc
    private double indexingTimeofFli(double boty, double botx) {
        double xdistance = getxdistance(boty,botx);
        double angle = Math.toRadians(idealAngle(boty, botx));
        double V = (idealpPow(boty, botx));
        return (xdistance / (V * Math.cos(angle))+delay);
    }
    public double indexingAng(double boty, double botx) {
        double xdistance = getxdistance(boty,botx);
        double timeOfFlight=indexingTimeofFli(boty,botx);
        double ydelta = goalHeight - turrHeight;
        double vy = (ydelta + 0.5 * 9.8 * timeOfFlight * timeOfFlight) / timeOfFlight;
        double vx = xdistance / timeOfFlight;
        double angle = Math.atan(vy / vx);
        return Math.toDegrees(angle);
    }
    private double indexingVel(double boty, double botx){
        double xdistance = getxdistance(boty,botx);
        double timeOfFlight=indexingTimeofFli(boty, botx);
        double ydelta = goalHeight - turrHeight;
        double vy = (ydelta + 0.5 * 9.8 * timeOfFlight * timeOfFlight) / timeOfFlight;
        double vx = xdistance / timeOfFlight;
        double V = Math.sqrt(vx*vx + vy*vy);
        return V;
//        double power=V/maxSpeed;
//        power=power+speedBoost;
//        if(V>=maxSpeed){
//            power=1;
//        }
//        return power;
    }
    //angler

    private double idealAngle(double boty, double botx){
        double xdistance = getxdistance(boty,botx);
        double ydelta=goalHeight-turrHeight;
        double angOfElev=Math.atan(ydelta/xdistance);
        double angle=(Math.toRadians(90)+angOfElev)/2;
        return Math.toDegrees(angle);
    }

    public void angler(double boty, double botx, boolean isIndexing){
        double angle=isIndexing?indexingAng(boty,botx):idealAngle(boty,botx);
        //TODO somehow convert angle to servo pos prob by graphing points os servos poses and their angle to get line of best fit
        double pos=angle/slope;//slope is what i would get from graphing points for example it could be 30deg/0.1 pos and if my angle is 60 and
        // divide by slope i get servos pos 0.2
        pos = Math.max(0.0, Math.min(1.0, pos)); //Make sure servo pos is between 0.0 and 1.0
        shotAngler.setPosition(pos);
    }
    //launcher
    private double idealpPow(double boty, double botx){
        double xdistance = getxdistance(boty,botx);
        double angle=Math.toRadians(idealAngle(boty,botx));
        double ydelta=goalHeight-turrHeight;
        double Vyt=(Math.tan(angle))*(xdistance);
        double xcomp=Math.sqrt(1/((ydelta-Vyt)/(0.5*(-9.8)*xdistance*xdistance)));
        double V=xcomp/Math.cos(angle);
        return V;
//        double power=V/maxSpeed;
//        power=power+speedBoost;
//        if(V>=maxSpeed){
//            power=1;
//        }
//        return power;
    }
    public void launchPow(double boty, double botx, boolean isIndexing) {
        double V = isIndexing ? indexingVel(boty, botx) : idealpPow(boty, botx);
        double RevolutionPerSec = (V / ((72 * Math.PI) / 1000));
        double velocity = ((RevolutionPerSec) * ticksPerRev);
        mainFlyWheel.setVelocity(velocity);
    }
    //aimer
    private double PID(int initPos,long targetPos,long time){
        long Error = targetPos-initPos;
        if(time<=0){
            time=1;
        }
        double errorChange=(Error-lastError)/time;
        ErrorSum+=(Error*time);
        lastError=Error;
        return ((kp*Error)+(ki*ErrorSum)+(kd*errorChange));
    }
    private long targetpos(double botx, double boty, double botTheta){// for this to work we need to start with turret at 90 degree
        double ticksPerAng=(ticksFor1Rotation/360.0);
        double xLength=(goalx-(botx*0.0254));//convert to meter
        double yLength=(ygoal-(boty*0.0254));
        double rawturrAngle=Math.toDegrees(Math.atan2(yLength,xLength));
        System.out.println("Turrangle with no robot heading: "+rawturrAngle);
        double turrAngle=-(rawturrAngle-botTheta);
        System.out.println("angle of turr: "+turrAngle);
        long ticks=Math.round(ticksPerAng*turrAngle);
        return ticks;
    }
    public void Aimer(double botx, double boty, double botTheta, long time){
        long Time=time-prevTime;
        prevTime=time;
        double pow=PID(turrMover.getCurrentPosition(),targetpos(botx, boty, botTheta),Time);
        turrMover.setPower(pow);
    }

    //color stuff
   /* public boolean fullCheck(){
        return colorSensor.getDistance(DistanceUnit.MM)<fullThreshold;
    }
    public String colorCheck(){
        String ballColor = "unknown";
        double raw_red = colorSensor.red();
        double raw_green = colorSensor.green();
        double raw_blue = colorSensor.blue();
        double max = Math.max(Math.max(raw_red, raw_green), raw_blue);
        int red=0;
        int green=0;
        int blue=0;
        if (max > 0) {
            red=(int)((raw_red/max)*255);
            green=(int)((raw_green/max)*255);
            blue=(int)((raw_blue/max)*255);
        }
        Color.RGBToHSV(red, green, blue,hsv);
        if(hsv[0]<greenMax&&hsv[0]>greenMin){
            ballColor="green";
        }
        else if(hsv[0]<purpleMax&&hsv[0]>purpleMin){
            ballColor="purple";
        }else{
            ballColor="unknown";
        }
        return ballColor;
    }*/
}
