package org.firstinspires.ftc.teamcode.subSystem;

import static org.firstinspires.ftc.teamcode.TeleOp.Cast_Ration.debugging;
import static org.firstinspires.ftc.teamcode.TeleOp.Cast_Ration.isRed;

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
public class LimelightTracking {
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    public Limelight3A limelight;
    private long lastTime;
    private long curTime;
    public static double turret_kp = 0.003;
    public static double turret_ki = 0;
    public static double turret_kd = 0;
    public static double turret_kf = 0.7;
    public static double friction = 0.12;
    public static double turret_integral_sum_limit = 1000;
    private double turret_lastError = 0;
    private double turret_errorSum = 0;
    public LLResult result;
    public double error = 0;
    public static double limit = 490;
    public boolean limiting=false;
    public double power=0;
    public double x=0;
    public double conversionRate = (2288.0/360.0);//tick per ang
    public double distance=1;
    public static double turretLeadTime = 0.3;
    public double xLength;
    public double yLength;
    public static boolean futTurr=true;
    public static boolean limited=false;
    Telemetry telemetry;
    public static double redxPos=144;
    private double goalPosx= isRed ?redxPos:9;
    private static double goalPosy=135;
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

    public double turret_PID(long curtime, double rightx) {
        double dtime = (curtime - lastTime) / 1000.0;
        lastTime = curtime;
        double error = this.error;
        if (dtime <= 0) {
            dtime = 1;
        }
        double errorChange = (error - turret_lastError) / dtime;
        if (!limiting) {
            turret_errorSum += (error * dtime);
            if (Math.abs(turret_errorSum) > turret_integral_sum_limit) {
                turret_errorSum = turret_errorSum < 0 ? -turret_integral_sum_limit : turret_integral_sum_limit;
            }
        }
        turret_lastError = error;
        double power = (turret_kp * error) + (turret_ki * turret_errorSum) + (turret_kd * errorChange) + (turret_kf * rightx);
        if (!(Math.abs(error) < 6.7)) power += power > 0 ? friction : -friction;
        return power;
    }

    public void manualTurret(double manualTurretPower) {
        Turret.setPower(manualTurretPower);
    }
    public Pose Localize(double heading) {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return new Pose(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y, heading);
        } else {
            return null;
        }
    }
    public void disableTurret() {
        Turret.setPower(0);
        telemetry.addData("turr Disabled !", " ");
    }

    public void updateTurret(double botHeading, double botXPos, double botYpos,double vx, double vy,double rightx,boolean isRed){
        curTime = System.currentTimeMillis();
        double turrPos=Turret.getCurrentPosition();
        curTime=System.currentTimeMillis();
        result = limelight.getLatestResult();
        botHeading=Math.toDegrees(botHeading);
        botHeading=botHeading>0?-180+botHeading:180+botHeading;//this make zero the back of bot not intake
        if(isRed) {
            limelight.pipelineSwitch(1);
        }else{
            limelight.pipelineSwitch(2);
        }
        if(futTurr){
            double futureX = botXPos + vx * turretLeadTime;
            double futureY = botYpos + vy * turretLeadTime;
            xLength = isRed?redxPos:14 - futureX;
            yLength = goalPosy - futureY;
        }else{
            xLength = (isRed?redxPos:14 - botXPos);//we know the goals are at the very most right or left so we subtract botx from goal to get our xlenght. we dont have absolute val to keep direction
            yLength = (goalPosy - botYpos);
        }
            double rawturrAngle = Math.toDegrees(Math.atan2(yLength, xLength));
            double turrAngle = (rawturrAngle - botHeading);
            while (turrAngle > 180) {
                turrAngle -= 360;
            }
            while (turrAngle < -180) {
                turrAngle += 360;
            }
            x = Math.round(turrAngle);//x here is the angle neeeded relative tothe bot
            if(x*conversionRate>limit){
                limited=true;
                x=limit*(1/(conversionRate));
            }else if(x*conversionRate<-limit){
                x=-limit*(1/(conversionRate));
                limited=true;
            }else{
                limited=false;
            }
            double turrCurPos = turrPos;
            x = x*conversionRate - turrCurPos;
            if(debugging) {
                telemetry.addData("raw ang", rawturrAngle);
                telemetry.addData("turrAngle", turrAngle);
                telemetry.addData("ideal turrPos", turrAngle * conversionRate);
                telemetry.addData("turr error", x);
            }
            error = x;
            power = turret_PID(curTime,rightx);


            if (turrPos > limit && power > 0) {
                power = 0;
                limiting = true;
            } else if (turrPos < -limit && power < 0) {
                power = 0;
                limiting = true;
            } else {
                limiting = false;
            }
            Turret.setPower(power);
        if(result!=null&&result.isValid()){//TODO in LL config set limelight pos as at the back of the bot and test this
            Pose3D botPose=result.getBotpose();
            double heading=turrPos*(360.0/2288.0)+result.getTx();
            heading=botPose.getOrientation().getYaw()-heading;
            Pose updatePose=new Pose(botPose.getPosition().x,botPose.getPosition().y,Math.toRadians(heading));
            if(debugging)telemetry.addData("LL botPose",updatePose);
        }
    }
    public boolean isTracked(){
        if(Math.abs(error)<10&&!limited){
            return true;
        }else{
            return false;
        }
    }

    public int shootingSpeed() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            distance = distanceAprilTag(llResult.getTa());
            telemetry.addData("limelight distance working (cm)", distance);
            return (int) (6553.762 - (167.7485 * distance) + (2.001088 * Math.pow(distance, 2)) - (0.01014018 * Math.pow(distance, 3)) + (0.00001876297 * Math.pow(distance, 4)));
        } else {
            return -4167;
        }
    }
    public void turrReset(){
        double turrPos=Turret.getCurrentPosition();
        error = turrPos;
        if(Math.abs(error)>10){
            turret_errorSum=0;
        }
        power = turret_PID(curTime, 0);
        if (turrPos>=limit&&-power>0){
            power=0;
            limiting=true;
        } else if (turrPos<=-limit&&-power<0){
            power=0;
            limiting=true;
        }else{
            limiting=false;
        }
        Turret.setPower(-power);
    }

    private double distanceAprilTag(double ta) {
        double scale = 30692.95;
        double distance = Math.sqrt(scale / ta) + 2;
        return distance;
    }
}
