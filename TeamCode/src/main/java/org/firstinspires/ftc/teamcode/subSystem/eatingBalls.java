package org.firstinspires.ftc.teamcode.subSystem;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;

public class eatingBalls {
    private DcMotorEx intake;
    private RevColorSensorV3 colorSensor;
    public static double intakePow=0.8;
    public static double outtakePow=-0.8;
    //color stuff
    public static double fullThreshold=10;
    public static double greenMin=100;
    public static double greenMax=150;
    public static double purpleMin=270;
    public static double purpleMax=330;
    public float[] hsv=new float[3];
    public static String[] ballsInside={"x","x","x"};
    public eatingBalls(LinearOpMode op){
        intake = op.hardwareMap.get(DcMotorEx.class,"intake");
        colorSensor = op.hardwareMap.get(RevColorSensorV3.class,"colorSensor");
    }
    //color sensor
    public void intaking(){
        intake.setPower(intakePow);
    }
    public void outtaking(){//idk when u would use this but maybe if u alr have 3 balls ig
        intake.setPower(outtakePow);
    }
    public boolean fullCheck(){
        return colorSensor.getDistance(DistanceUnit.MM)<fullThreshold;
    }
    public String colorCheck(){
        String ballColor = "unknown";
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(),  colorSensor.blue(),hsv); //scale color sensor values to 0-255
        if(hsv[0]<greenMax&&hsv[0]>greenMin){
            ballColor="green";
        }
        else if(hsv[0]<purpleMax&&hsv[0]>purpleMin){
            ballColor="purple";
        }else{
            ballColor="unknown";
        }
        return ballColor;
    }
}
