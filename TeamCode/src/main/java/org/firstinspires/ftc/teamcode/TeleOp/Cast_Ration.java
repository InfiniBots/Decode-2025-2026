package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Autonomous.StorePos.curPose;
import static org.firstinspires.ftc.teamcode.Autonomous.blueNearAuto.b_finalShoot;
import static org.firstinspires.ftc.teamcode.Autonomous.redGoalAuto.finalShoot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subSystem.LimelightTracking;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class Cast_Ration extends LinearOpMode {
    public static boolean isRed =true;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx IntakeMotor;
    private DcMotorEx Turret;
    private VoltageSensor Voltage;
    public LimelightTracking lltracking;
    public double intakeCurrent=0;
    public double frontLeft;
    public double frontRight;
    public double backLeft;
    public double backRight;
    public static double kp = 0.003;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0;
    private double lastError = 0;
    private double errorSum = 0;
    private long lastTime = 0;
    private DcMotorEx TopFlywheel;
    private DcMotorEx BottomFlywheel;
    public int ticksPerSecond = 0;
    public double goalVel;
    public long loopTime=80;
    public long loopStart;
    public static int stopperThreshold = 80;
    public static double emptyThreshold = 700;//TODO edit this variable
    public static boolean tracking=true;
    private long currTime;
    private long deltaTime;
    private Servo Stopper1;
    private Servo Stopper2;
    public ArrayList<Double> cycles = new ArrayList<>();
    public long cycleTime;
    public double cycleAvg = 0;
    public boolean turretzeroing=false;
    public static boolean usePedroMode = false;
    private Follower follower;
    public double heading;
    public double Xpos;
    public double Ypos;
    public static int shootingSpeed = 1500;
    public double distanceToGoal;
    public double timef;
    public double futx;
    public double futy;
    public double lastGoalD;
    public int custom_tp;
    public boolean equationDisabled;
    public boolean shooterOn=false;
    public static boolean autoShoot=true;
    public boolean slowShot=true;
    public double TopVelocity;
    public double bottomVelocity;
    public double Sensitivity;
    public static double gSensitivity=0.85;
    public static double pSensitivity=0.79;
    private boolean lastBack = false;
    private boolean lastB = false;
    private boolean prevRightStickButton = false;
    public static boolean continueing=false;
    public boolean ytoggle=false;
    public boolean gettingTailgatedBySomeExodiusTypeBot = false;
    public double stationaryTPS = 283.2006 + (65.59412 * distanceToGoal) - (1.299762 * (distanceToGoal * distanceToGoal)) + (0.01202799 * Math.pow(distanceToGoal, 3)) - (0.00003992315 * Math.pow(distanceToGoal, 4));
    enum State {
        GENERAL_MOVEMENT,
        PEW_PEW

    }
    State state;
    public boolean once=true;
    public boolean turrToggle=true;
    public boolean turretOnOff =true;
    public boolean PewPewActive=false;
    public int intakeReads=0;
    public ArrayList<Double> intakeReadings=new ArrayList<>();
    public double intakeAvg=1;
    public static double chillSpeed=-1;
    public static double intakeMult=100;
    public long intakespikeDelay;
    public static double intakespikeThresh=500;
    /*public PathChain park;
    public void buildPath(){
        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(follower.getPose(), (isRed ?(new Pose(38.7,33.3)):(new Pose(105.3,33.3))))
                )
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-90))
                .build();
    }*/

    public double PID(double currentVelocity, double targetVelocity, long time) {
        double error = targetVelocity - currentVelocity;
        telemetry.addData("targetVelocity",targetVelocity);
        if (time <= 0) {
            time = 1;
        }
        double errorChange = (error - lastError) / time;
        errorSum += (error * time);
        lastError = error;
        return ((kp * error) + (ki * errorSum) + (kd * errorChange) + ((0.0007448464 - (3.3333219e-7 * targetVelocity) + (8.791839e-11 * targetVelocity * targetVelocity)) * targetVelocity));//added new velocity thingy

    }


    public double computeMovingCompensatedTPS(double distanceToGoal, Pose robotPose, Vector robotVelocity, boolean isRed){

        double goalX = isRed ? 130 : 14;
        double goalY = 135;

        double vx = robotVelocity.getXComponent();
        double vy = robotVelocity.getYComponent();

        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        double dist = Math.sqrt(dx*dx + dy*dy);


        double ux = dx / dist;
        double uy = dy / dist;

        double v_robot = vx * ux + vy * uy;

        double stationaryTPS = 283.2006 + (65.59412 * distanceToGoal) - (1.299762 * (distanceToGoal * distanceToGoal)) + (0.01202799 * Math.pow(distanceToGoal, 3)) - (0.00003992315 * Math.pow(distanceToGoal, 4));

        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("dist, ", dist);
        telemetry.addData("ux", ux);
        telemetry.addData("uy, ", uy);
        telemetry.addData("v_robot", v_robot);
        telemetry.addData("velocity x", robotVelocity.getXComponent());
        telemetry.addData("velocity y,", robotVelocity.getYComponent());

        double wheelCircumferenceInches = Math.PI * (72.0/25.4); //wheel diameter in inches
        double ticksPerInch = 28.0/wheelCircumferenceInches; //motor ticks per rev
        return stationaryTPS - ticksPerInch * v_robot;

    }


    public void updateShooter() {
        deltaTime = currTime - lastTime;
        if (gamepad2.b && !lastB){
            equationDisabled = !equationDisabled;
        }
        lastB = gamepad2.b;
        if(shooterOn) {
            double power1;
            if (equationDisabled||slowShot){
                power1 = PID(Math.max(TopVelocity, bottomVelocity), ticksPerSecond, deltaTime) * (12.0 / Voltage.getVoltage());
                power1 = Math.max(-1.0, Math.min(1.0, power1));
            } else {
                power1 = PID(Math.max(TopVelocity, bottomVelocity), custom_tp, deltaTime) * (12.0 / Voltage.getVoltage());
                power1 = Math.max(-1.0, Math.min(1.0, power1));
            }
            lastTime = currTime;

            TopFlywheel.setPower(-power1);
            BottomFlywheel.setPower(-power1);
        }else{
            TopFlywheel.setPower(0);
            BottomFlywheel.setPower(0);
        }
    }
    public void updateDistanceToGoal_stationaryTPS(){
        double xRobot = follower.getPose().getX();
        double yRobot = follower.getPose().getY();
        distanceToGoal = isRed?Math.sqrt(Math.pow((130-xRobot), 2) + Math.pow((135-yRobot), 2)):Math.sqrt(Math.pow((14-xRobot), 2) + Math.pow((135-yRobot), 2));
        stationaryTPS =283.2006 + (65.59412 * distanceToGoal) - (1.299762 * (distanceToGoal * distanceToGoal)) + (0.01202799 * Math.pow(distanceToGoal, 3)) - (0.00003992315 * Math.pow(distanceToGoal, 4));
        timef=1;//input whatever equation here
    }
    public void bulkRead(){
        Xpos=follower.getPose().getX();
        Ypos=follower.getPose().getY();
        heading=follower.getHeading();
        TopVelocity=TopFlywheel.getVelocity();
        bottomVelocity=BottomFlywheel.getVelocity();
        intakeCurrent=IntakeMotor.getCurrent(CurrentUnit.AMPS);
    }
    public boolean inZone(double x,double y){
        if((y>=x&&y>=-x+144)||(y<=x-48&&y<=-x+96)){
            return true;
        }else{
            return false;
        }
    }
    public boolean isEmpty(){
        if(intakeAvg<emptyThreshold){
            return true;
        }else {
            return false;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(curPose);
        follower.update();




        lltracking = new LimelightTracking(this,telemetry);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Voltage = hardwareMap.voltageSensor.iterator().next();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        TopFlywheel = hardwareMap.get(DcMotorEx.class, "TopFlywheel");
        BottomFlywheel = hardwareMap.get(DcMotorEx.class, "BottomFlywheel");
        TopFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        Stopper1 = hardwareMap.get(Servo.class, "Stopper1");
        //Stopper2 = hardwareMap.get(Servo.class, "Stopper2");

        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//TODO temporary for testing purposes
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = State.GENERAL_MOVEMENT;

        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.update();
        waitForStart();
        lastTime = System.currentTimeMillis();
        cycleTime = System.currentTimeMillis();
        intakespikeDelay=System.currentTimeMillis();
        follower.update();


        while (opModeIsActive()) {
            currTime = System.currentTimeMillis();
            loopStart = currTime;
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            bulkRead();
            updateDistanceToGoal_stationaryTPS();
            follower.update();



            double totalCurrentAmps = controlHub.getCurrent(CurrentUnit.AMPS);

            double expansionCurrentAmps = expansionHub.getCurrent(CurrentUnit.AMPS);
            if(gamepad2.x&& turretOnOff){
                tracking=!tracking;
                turretOnOff =false;
            }else if(!gamepad2.x){
                turretOnOff =true;
            }

            /*if(gamepad1.x&&Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.right_stick_x)+Math.abs(gamepad1.left_stick_y)==0){
                if(once) {
                    buildPath();
                    once=false;
                    follower.followPath(park);
                }

            }else if (!gamepad1.x) {
                once = true;*/


                double x = gamepad1.left_stick_x * 1.1;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x * Sensitivity;

                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                 frontLeft = power * cos / max + turn;
                 frontRight = power * sin / max - turn;
                 backLeft = power * sin / max + turn;
                 backRight = power * cos / max - turn;

                if ((power + Math.abs(turn)) > 1) {
                    frontLeft /= power + Math.abs(turn);
                    frontRight /= power + Math.abs(turn);
                    backLeft /= power + Math.abs(turn);
                    backRight /= power + Math.abs(turn);
                }
           /* if (totalCurrentAmps >= 600613){ // i have no clue what the threshold for the amps taken by the drivetrain is to the point where it stops other motors from functioning at full speed so i just put google as a placeholder this should change hopefully in the near future
                gettingTailgatedBySomeExodiusTypeBot = true; // :(
                frontRight *= 0.75; // it should in theory keep doing 75% of its full power until it reaches the threshold keyword in theory
                frontLeft *= 0.75; // it should in theory keep doing 75% of its full power until it reaches the threshold keyword in theory
                backRight*= 0.75; // it should in theory keep doing 75% of its full power until it reaches the threshold keyword in theory
                backLeft *= 0.75; // it should in theory keep doing 75% of its full power until it reaches the threshold keyword in theory
            } else {
                gettingTailgatedBySomeExodiusTypeBot = false; // :D
            }
            ts is NOT the play*/

                frontLeftMotor.setPower(frontLeft);
                backLeftMotor.setPower(backLeft);
                frontRightMotor.setPower(frontRight);
                backRightMotor.setPower(backRight);
                if(gamepad2.y&&!ytoggle){
                    follower.setPose(new Pose(7,11,Math.toRadians(-90)));
                }
                ytoggle = gamepad2.y;

                if (gamepad2.back && !lastBack) {
                    isRed = !isRed;
                    follower.setPose(isRed?finalShoot:b_finalShoot);
                }
            lastBack = gamepad2.back;
            /*futx=follower.getVelocity().getXComponent();
            futy=follower.getVelocity().getYComponent();
            double xLength = (isRed?130:14 - Xpos);//we know the goals are at the very most right or left so we subtract botx from goal to get our xlenght. we dont have absolute val to keep direction
            double yLength = (135 - Ypos);
            double angGoal =Math.atan2(yLength, xLength);
            double newv=futx*Math.cos(angGoal)+futy*Math.sin(angGoal);
            double futDistanceToGoal=distanceToGoal-newv*timef;*/

            switch (state) {
                case GENERAL_MOVEMENT:
                    slowShot=true;
                    intakeReadings.add(IntakeMotor.getCurrent(CurrentUnit.AMPS));
                    if(intakeReadings.size()>3){
                        intakeReadings.remove(0);
                    }
                    //intake avg
                    for(int i=0;i<intakeReadings.size();i++){
                        if(i==0){
                            intakeAvg=0;
                        }
                        intakeAvg+=intakeReadings.get(0);
                        if(i==intakeReadings.size()-1)intakeAvg=intakeAvg*intakeMult;
                    }
                    shooterOn=false;
                    Sensitivity=gSensitivity;
                    PewPewActive=false;
                    custom_tp = 0;//TODO  make slow spin also prob need to change shooterOn for that
                    ticksPerSecond = 0;
                    if (gamepad1.left_trigger > 0.1) {
                        IntakeMotor.setPower(1);
                    } else if (gamepad1.right_trigger > 0.1) {
                        IntakeMotor.setPower(-1);
                    } else {
                        IntakeMotor.setPower(chillSpeed);
                    }
                    lltracking.turrReset();

                    if (gamepad2.dpad_left) {
                        Turret.setPower(-0.5);
                        tracking=false;
                    } else if (gamepad2.dpad_right) {
                        Turret.setPower(0.5);
                        tracking=false;
                    }  else if(!tracking){
                        Turret.setPower(0);
                    }

                    if (gamepad1.right_bumper) {
                        state = State.PEW_PEW;
                        PewPewActive=false;
                        continueing=true;
                    }
                    if(autoShoot&&!isEmpty()&&currTime-intakespikeDelay>=intakespikeThresh){
                        state = State.PEW_PEW;
                        PewPewActive=true;
                    }

                    if (gamepad2.dpad_up) {
                        Stopper1.setPosition(1);
                        //Stopper2.setPosition(1);
                    } else if (gamepad2.dpad_down) {
                        Stopper1.setPosition(0);
                        //Stopper2.setPosition(0);
                    } else {
                        Stopper1.setPosition(0.767);
                        //Stopper2.setPosition(0.62);
                    }

                    break;

                case PEW_PEW:
                    slowShot=false;
                    intakeReadings.add(IntakeMotor.getCurrent(CurrentUnit.AMPS));
                    if(intakeReadings.size()>3){
                        intakeReadings.remove(0);
                    }
                    //intake avg
                    for(int i=0;i<intakeReadings.size();i++){
                        if(i==0){
                            intakeAvg=0;
                        }
                        intakeAvg+=intakeReadings.get(0);
                        if(i==intakeReadings.size()-1)intakeAvg=intakeAvg*intakeMult;
                    }

                    Sensitivity=pSensitivity;
                    custom_tp = (int) computeMovingCompensatedTPS(distanceToGoal, follower.getPose(), follower.getVelocity(), isRed);
                    shooterOn=true;
                    if(tracking)lltracking.updateTurret(heading,Xpos, Ypos,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(),gamepad1.right_stick_x, isRed);
                    //ticksPerSecond = lltracking.shootingSpeed()!=-4167?lltracking.shootingSpeed()-20:1500;
                    ticksPerSecond = shootingSpeed;
                    if (gamepad2.a) {
                        Stopper1.setPosition(1);
                        // Stopper2.setPosition(1);
                    }
                    if(autoShoot) {
                        if (isEmpty() || !inZone(Xpos, Ypos)) {
                            PewPewActive = false;
                        } else if (!isEmpty() || inZone(Xpos, Ypos)) {
                            PewPewActive = true;
                        }
                    }
                    if (gamepad1.left_bumper) {
                        double cyc = (currTime - cycleTime) / 1000;
                        cycles.add(cyc);
                        cycleTime = currTime;
                        ticksPerSecond = 0;
                        PewPewActive=false;
                        state = State.GENERAL_MOVEMENT;
                        intakeReadings.clear();
                    }
                    if(gamepad1.right_bumper&&!continueing) {
                        PewPewActive = true;
                        Sensitivity=pSensitivity;
                    }else if(!gamepad1.right_bumper){
                        continueing=false;
                    }

                    if(PewPewActive) {
                        IntakeMotor.setPower(-1);
                    }else{
                        IntakeMotor.setPower(chillSpeed);
                    }

                    if (gamepad2.dpad_up) {
                        Stopper1.setPosition(1);
                      //  Stopper2.setPosition(1);
                    } else if (gamepad2.dpad_down) {
                        Stopper1.setPosition(0);
                       // Stopper2.setPosition(0);
                    } else {
                        if(!equationDisabled) {
                            if (PewPewActive && Math.abs(Math.max(TopVelocity,bottomVelocity) - custom_tp) < stopperThreshold&& (lltracking.isTracked()||!tracking)) {
                                Stopper1.setPosition(1);
                                // Stopper2.setPosition(1);
                            }
                        }else{
                            if (PewPewActive&&(Math.abs(Math.max(TopVelocity,bottomVelocity) - ticksPerSecond)) < stopperThreshold&& (lltracking.isTracked()||!tracking)) {
                                Stopper1.setPosition(1);
                                // Stopper2.setPosition(1);
                            }
                        }
                    }
                    break;


            }
            updateShooter();
            telemetry.addData("pewpewActive",PewPewActive);
            telemetry.addData("intakeAVG",intakeAvg);
            telemetry.addData("isempty",isEmpty());
            telemetry.addData("inZOne",inZone(Xpos,Ypos));
            telemetry.addData("Custom TPS",custom_tp);
            telemetry.addData("leftRear",backLeft);
            telemetry.addData("State: ", state);
            telemetry.addData("TopFlywheel Velocity", TopVelocity);
            telemetry.addData("BottomFlywheel Velocity", bottomVelocity);
            telemetry.addData("Target Speed", ticksPerSecond);
            telemetry.addData("Error", ticksPerSecond - TopVelocity);
            telemetry.addData("Current of shooter", TopFlywheel.getCurrent(CurrentUnit.AMPS) + BottomFlywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Control Hub Current: ", totalCurrentAmps);
            telemetry.addData("Expansion Hub 2: ", expansionCurrentAmps);
            telemetry.addData("intake current", intakeCurrent);
            telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            telemetry.addData("Position of Robot X: ", Xpos);
            telemetry.addData("Position of Robot Y: ", Ypos);
            telemetry.addData("Distance to  Goal: ", distanceToGoal);
            telemetry.addData("Pipeline: ", lltracking.limelight.getStatus().getPipelineIndex());
            telemetry.addData("EquationDisabled: ", equationDisabled);
            telemetry.addData("rearleft dt val",backLeftMotor);
            telemetry.addData("difference of velocity: ",  computeMovingCompensatedTPS(distanceToGoal, follower.getPose(), follower.getVelocity(), isRed) - stationaryTPS);
            telemetry.addData("full thing: ", computeMovingCompensatedTPS(distanceToGoal, follower.getPose(), follower.getVelocity(), isRed));
            telemetry.addData("looptime: ", loopTime);
            telemetry.update();
            curPose=follower.getPose();
            loopTime = System.currentTimeMillis() - loopStart;
        }
        for (int i = 0; i < cycles.size(); i++) {
            telemetry.addData("Cycle " + (i + 1) + ": ", cycles.get(i));
            cycleAvg += cycles.get(i);
            telemetry.update();
        }
        telemetry.addData("Cycle avg: ", cycleAvg / cycles.size());
        telemetry.update();
    }
}