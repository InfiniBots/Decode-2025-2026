package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    public Servo Stopper1;
    public Servo Stopper2;
    public static double stop1=0.77;
    public static double stop2=0.7;
    public static double syncpos=0.5;
    public static boolean close=false;
    public static double syncOffset=0;

    @Override
    public void runOpMode() {
        Stopper1=hardwareMap.get(Servo.class,"Stopper1");
        Stopper2=hardwareMap.get(Servo.class,"Stopper2");
        waitForStart();
        while (opModeIsActive()){

//close is 0.62 and 0.56(limelight side)
// open is 0.77 and 0.7
            Stopper1.setPosition(stop1);
            Stopper2.setPosition(stop2);

        }
    }
}
