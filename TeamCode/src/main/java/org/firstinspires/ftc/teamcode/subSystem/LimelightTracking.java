package org.firstinspires.ftc.teamcode.subSystem;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.TeleOp.Cast_Ration.isRed;
import static org.firstinspires.ftc.teamcode.TeleOp.Robot.issRED;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class LimelightTracking{
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    private double targetPosition = 0;
    private boolean isntGettingRecognized = false;
    public Limelight3A limelight;
    private long lastTime;
    private long curTime;
    public static double turret_kp = 0.003;
    public static double turret_ki = 0;
    public static double turret_kd = 0;
    public static double turret_kf=0.7;
    public static double friction=0.12;
    public static double turret_integral_sum_limit = 1000;
    private double turret_lastError = 0;
    private double turret_errorSum = 0;
    public static double ticksPRotation = 2288;
    public double ticksPerAng=1;
    private final double wishingX = 0.00;
    public LLResult result;
    public double error=0;
    public static double limit = 490;
    public boolean limiting=false;
    public double power=0;
    public double x=0;
    public double conversionRate = (2288.0/360.0);//tick per ang
    public int llostThres=1000;
    public long lockeddelay;
    public double distance=1;
    Telemetry telemetry;
    private double goalPosx= isRed ?135:9;
    private double goalPosy=135;
    public LimelightTracking(LinearOpMode op, Telemetry telemetry){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret = op.hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);
        lastTime = System.currentTimeMillis();
        limelight = op.hardwareMap.get(Limelight3A.class, "limelight");
        Voltage = op.hardwareMap.voltageSensor.iterator().next();
        limelight.start();
    }

    public double turret_PID(long curtime,double rightx) {
        if(isRed) {
            limelight.pipelineSwitch(1);
        }else{
            limelight.pipelineSwitch(2);
        }
        double dtime = (curtime - lastTime) / 1000.0;
        lastTime = curtime;
        double error = this.error;
        if (dtime <= 0) {
            dtime = 1;
        }
        double errorChange = (error - turret_lastError) / dtime;
        if(!limiting) {
            turret_errorSum += (error * dtime);
            if (Math.abs(turret_errorSum) > turret_integral_sum_limit) {
                turret_errorSum = turret_errorSum<0? -turret_integral_sum_limit: turret_integral_sum_limit;
            }
        }
        turret_lastError = error;
        double power=((turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange)+(turret_kf*rightx));
        if(!(Math.abs(error)<6.7))power+=power>0?friction:-friction;
        return power;
    }
    public void manualTurret(double manualTurretPower){
        Turret.setPower(manualTurretPower);
    }
    public Pose Localize(double heading){
        result = limelight.getLatestResult();
        if(result!=null&&result.isValid()){
            return(new Pose(result.getBotpose().getPosition().x,result.getBotpose().getPosition().y,heading));
        }else {
            return null;
        }
    }

    public void disableTurret(){
        Turret.setPower(0);
        telemetry.addData("turr Disabled !", " ");
    }

    public void updateTurret(double botHeading, double botXPos, double botYpos,double rightx){
        curTime=System.currentTimeMillis();
        result = limelight.getLatestResult();
        botHeading=Math.toDegrees(botHeading);
        botHeading=botHeading>0?-180+botHeading:180+botHeading;//this make zero the back of bot not intake
        telemetry.addData("botHeading",botHeading);

        if (result == null || !result.isValid()){
            isntGettingRecognized = true;
              /*
                double xLength = (goalPosx - botXPos);//we know the goals are at the very most right or left so we subtract botx from goal to get our xlenght. we dont have absolute val to keep direction
                double yLength = (goalPosy - botYpos);
                double rawturrAngle = Math.toDegrees(Math.atan2(yLength, xLength));
                telemetry.addData("raw ang", rawturrAngle);
                double turrAngle = (rawturrAngle - botHeading);
                while (turrAngle > 180) {
                    turrAngle -= 360;
                }
                while (turrAngle < -180) {
                    turrAngle += 360;
                }
                x = Math.round(turrAngle);//x here is the angle neeeded relative tothe bot
                double turrCurPos = Turret.getCurrentPosition();
                x = x*conversionRate - turrCurPos;
                telemetry.addData("turrAngle", turrAngle);
                telemetry.addData("ideal turrPos", x);
                if(x>limit){
                    x=limit;
                }else if(x<-limit){
                    x=-limit;
                }
                error = x;
                power = turret_PID(curTime,rightx);*/


            if (Turret.getCurrentPosition() > limit && power > 0) {
                power = 0;
                limiting = true;
            } else if (Turret.getCurrentPosition() < -limit && power < 0) {
                power = 0;
                limiting = true;
            } else {
                limiting = false;
            }
            Turret.setPower(0);
        } else {
            lockeddelay=curTime;
            isntGettingRecognized=false;
            x = result.getTx();
            error=(x*((double) 2288 /360));
            power = turret_PID(curTime,rightx);
            if (Turret.getCurrentPosition()>=limit&&power>0){
                power=0;
                limiting=true;
            } else if (Turret.getCurrentPosition()<=-limit&&power<0){
                power=0;
                limiting=true;
            }else{
                limiting=false;
            }
            Turret.setPower(power);



        }
        telemetry.addData("botHeading",botHeading);
        telemetry.addData("botxpos",botXPos);
        telemetry.addData("botypos",botYpos);
        telemetry.addData("isn'tGettingRecognized: ", isntGettingRecognized);
        telemetry.addData("target x: ", x);
        telemetry.addData("error: ", error);
        telemetry.addData("targetPOS",targetPosition);
        telemetry.addData("limiting",limiting);
        telemetry.addData("isn'tGettingRecognized: ", isntGettingRecognized);
        telemetry.addData("currpos",Turret.getCurrentPosition());
        telemetry.update();

    }
                                                                                                
    public int shootingSpeed(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null&&llResult.isValid()) {
            distance = distanceAprilTag(llResult.getTa());
            telemetry.addData("limelight distance working (cm)",distance);
            return (int)(6553.762-(167.7485*distance)+(2.001088*Math.pow(distance,2))-(0.01014018*Math.pow(distance,3))+(0.00001876297*Math.pow(distance,4)));
        }else{
            return -4167;
        }
    }
    public void turrReset(){
        error = Turret.getCurrentPosition();
        telemetry.addData("null error", error);
        power = turret_PID(curTime, 0);
        if (Turret.getCurrentPosition()>=limit&&-power>0){
            power=0;
            limiting=true;
        } else if (Turret.getCurrentPosition()<=-limit&&-power<0){
            power=0;
            limiting=true;
        }else{
            limiting=false;
        }
        Turret.setPower(-power);
        telemetry.addData("null power",Turret.getPower());
    }


    private double distanceAprilTag(double ta) {
        double scale = 30692.95;
        double distance = Math.sqrt(scale/ta) + 2;
        return distance;
    }



}


