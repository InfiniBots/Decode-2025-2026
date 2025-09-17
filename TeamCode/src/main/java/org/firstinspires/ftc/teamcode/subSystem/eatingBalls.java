package org.firstinspires.ftc.teamcode.subSystem;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;

public class eatingBalls {
    private DcMotorEx intake;
    public static double intakePow=0.8;
    public static double outtakePow=-0.8;

    public eatingBalls(LinearOpMode op){
        intake = op.hardwareMap.get(DcMotorEx.class,"intake");
    }
    //color sensor
    public void intaking(){
        intake.setPower(intakePow);
    }
    public void outtaking(){//idk when u would use this but maybe if u alr have 3 balls ig
        intake.setPower(outtakePow);
    }
}
