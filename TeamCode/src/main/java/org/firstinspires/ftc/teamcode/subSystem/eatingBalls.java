package org.firstinspires.ftc.teamcode.subSystem;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;

public class eatingBalls {
    private DcMotorEx intakeStage1;
    private DcMotorEx intakeStage2;
    private Servo intakeStop;
    public Telemetry telemetry;

    public static double intakePow=0.8;
    public static double outtakePow=-0.8;
    public static double holdBalls=0.1;
    public static double intakeClosePos=0.5;
    public static double intakeOpenPos=0.5;

    public eatingBalls(LinearOpMode op, Telemetry telemetry){
        this.telemetry=telemetry;
        intakeStage1 = op.hardwareMap.get(DcMotorEx.class,"intake1");
        intakeStage2 = op.hardwareMap.get(DcMotorEx.class,"intake2");
        intakeStop = op.hardwareMap.get(Servo.class,"intakeStop");
    }
    public void intaking(){
        intakeStage1.setPower(intakePow);
        intakeStage2.setPower(intakePow);
    }
    public void outtaking(){//idk when u would use this but maybe if u alr have 3 balls ig
        intakeStage1.setPower(outtakePow);
        intakeStage2.setPower(outtakePow);
    }
    public void holdintakenBalls(){//after intaking we hold the balls in
        intakeStage1.setPower(holdBalls);
        intakeStage2.setPower(holdBalls);
    }
    public void intakeClose(){//stop intaken balls going into shooter
        intakeStop.setPosition(intakeClosePos);
    }
    public void intakeOpen(){//allow intaken balls going into shooter
        intakeStop.setPosition(intakeOpenPos);
    }
}
