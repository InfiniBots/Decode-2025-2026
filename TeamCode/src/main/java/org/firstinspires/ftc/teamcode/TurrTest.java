package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
@Config
public class TurrTest extends LinearOpMode {
    private DcMotorEx TurrMotor;
    private DcMotorEx intakeStage1;
    private DcMotorEx intakeStage2;
    private DcMotorEx TurrMotor2;
    public static double pow=1;
    public static boolean powering=false;
    public static double velocity_MperS=2;//ignore this for now
    public static double intakePow=1;
    public static int ticksPerSecond=700;
    private double velocity;
    //pid
    public static boolean pid=false;
    private double lastError=0;
    private double ErrorSum=0;
    private long lastTime=0;
    public static double kp=0;
    public static double kd=0;
    public static double ki=0;
    private double PID(double initPos,double targetPos,long time){
        double Error = targetPos-initPos;
        if(time<=0){
            time=1;
        }
        double errorChange=(Error-lastError)/time;
        ErrorSum+=(Error*time);
        lastError=Error;
        return ((kp*Error)+(ki*ErrorSum)+(kd*errorChange));
    }

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());//telem stuff
        TurrMotor2 = hardwareMap.get(DcMotorEx.class,"Motor1");
        TurrMotor = hardwareMap.get(DcMotorEx.class,"Motor0");
        //intakeStage1 = hardwareMap.get(DcMotorEx.class,"Motor0");
        //intakeStage2= hardwareMap.get(DcMotorEx.class,"Motor1");
        TurrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//gets encoder stuff
        TurrMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//gets encoder stuff
        TurrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        lastTime=System.currentTimeMillis();
        while (opModeIsActive()) {
            long curTime = System.currentTimeMillis();
           /* if(gamepad1.b){
                intakeStage1.setPower(intakePow);//sets intake power
                intakeStage2.setPower(intakePow);//sets intake power
            }else{
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);
            }*/

            if(powering) {//when u want to use power instead of velocity
                if (gamepad1.a) {
                    TurrMotor.setPower(pow);//set power when press a
                   // TurrMotor2.setPower(pow);
                } else {
                    TurrMotor.setPower(0);// sets to 0 when not pressed
                   // TurrMotor2.setPower(0);
                }
            }else if(!powering){
                if (gamepad1.a) { //castrate gautam
                    telemetry.addData("in",1);
                    if (pid){
                        TurrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        TurrMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//gets encoder stuff
                        long deltaTime=curTime-lastTime;
                        lastTime=curTime;
                        double pow=PID(TurrMotor.getVelocity(),ticksPerSecond,deltaTime);
                        pow=(pow+1)/2;
                        TurrMotor.setPower(-pow);
                        TurrMotor2.setPower(-pow);
                    }else {
                        TurrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        TurrMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        TurrMotor.setVelocity(ticksPerSecond);
                        TurrMotor2.setVelocity(ticksPerSecond);
                        //  TurrMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//gets encoder stuff

                      //  TurrMotor2.setVelocity(ticksPerSecond);
                    }
                } else {
                    TurrMotor.setVelocity(0);// sets to 0 when not pressed
                    TurrMotor2.setVelocity(0);// sets to 0 when not pressed
                    telemetry.addData("out",0);
                   // TurrMotor2.setVelocity(0);
                  //  TurrMotor.setPower(0);// sets to 0 when not pressed
                   // TurrMotor2.setPower(0);
                }
            }
            //telemtry
            telemetry.addData("TurrMotor velocity",TurrMotor.getVelocity());
            telemetry.addData("TurrMotor currTicks: ", TurrMotor.getCurrentPosition());
            telemetry.addData("TurrMotor 2 currTicks: ", TurrMotor2.getCurrentPosition());
            telemetry.addData("TurrMotor 2 velocity", TurrMotor2.getVelocity());
            telemetry.addData("Target speed",ticksPerSecond);
            telemetry.addData("Voltage",TurrMotor.getCurrent(CurrentUnit.AMPS)+TurrMotor2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("power",TurrMotor.getPower());
            PanelsTelemetry.INSTANCE.getTelemetry().addData("hi",1);
            PanelsTelemetry.INSTANCE.getTelemetry().update();
            telemetry.update();
        }
    }
}