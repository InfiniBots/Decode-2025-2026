package org.firstinspires.ftc.teamcode.subSystem;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;

public class eatingBalls {
    private DcMotorEx intake;
    private Servo intakeStop1;
    private Servo intakeStop2;
    public Telemetry telemetry;

    public static double intakePow=-1;
    public static double outtakePow=1;
    public static double intakeClosePos1=0.62;
    public static double intakeOpenPos1=0.77;
    public static double intakeClosePos2=0.56;
    public static double intakeOpenPos2=0.7;

    public eatingBalls(LinearOpMode op, Telemetry telemetry){
        this.telemetry=telemetry;
        intake = op.hardwareMap.get(DcMotorEx.class,"Intake");
        intakeStop1 = op.hardwareMap.get(Servo.class,"Stopper1");
        intakeStop2 = op.hardwareMap.get(Servo.class,"Stopper2");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void intaking(){
        intake.setPower(intakePow);
    }
    public void outtaking(){//idk when u would use this but maybe if u alr have 3 balls ig
        intake.setPower(outtakePow);
    }
    public void chilling(){//intake not moving
        intake.setPower(0);
    }
    public void intakeClose(){//stop intaken balls going into shooter
        intakeStop1.setPosition(intakeClosePos1);
        intakeStop1.setPosition(intakeClosePos2);
    }
    public void intakeOpen(){//allow intaken balls going into shooter
        intakeStop1.setPosition(intakeOpenPos1);
        intakeStop1.setPosition(intakeOpenPos2);
    }
}
