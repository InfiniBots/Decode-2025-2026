package org.firstinspires.ftc.teamcode.subSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.control.builder.ControlSystemBuilderKt;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
@Config
public class FlywheelVelocityPID implements Subsystem {

    private final MotorGroup motors;
    private final MotorEx topFlywheel;
    private final MotorEx bottomFlywheel;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private ControlSystem controlSystem;

    public static double kP = 0.0001;
    public static double kD = 0.0;
    public static double kV = 0.0002;
    public static double velocityTolerance = 20.0;
    private static final double ticksPerRev = 28.0;

    public FlywheelVelocityPID(HardwareMap hardwareMap, Telemetry opModeTelemetry) {
        topFlywheel = new MotorEx("TopFlywheel");
        bottomFlywheel = new MotorEx("BottomFlywheel").reversed();
        motors = new MotorGroup(topFlywheel, bottomFlywheel);
        this.telemetry = new MultipleTelemetry(opModeTelemetry, dashboard.getTelemetry());
        rebuildControlSystem();
    }

    private void rebuildControlSystem() {
        controlSystem = ControlSystemBuilderKt.controlSystem(builder -> {
            builder.velPid(kP, 0.0, kD);
            builder.basicFF(kV, 0.0, 0.0);
            return null;
        });
    }

    @Override
    public void periodic() {
        motors.setPower(controlSystem.calculate(motors.getState()));

        updateTelemetry();
    }

    private void updateTelemetry() {
        double targetVel = controlSystem.getGoal().getVelocity();
        double currentVel = motors.getState().getVelocity();
        double targetRPM = ticksToRpm(targetVel);
        double currentRPM = ticksToRpm(currentVel);
        double error = targetVel - currentVel;
        double power = motors.getPower();
        double topFlywheelCurrent = topFlywheel.getMotor().getCurrent(CurrentUnit.AMPS);
        double bottomFlywheelCurrent = bottomFlywheel.getMotor().getCurrent(CurrentUnit.AMPS);
        boolean atTarget = atTargetVelocity();

        telemetry.addData("=== FLYWHEEL ===", "");
        telemetry.addData("Target Velocity", "%.1f TPS (%.1f RPM)", targetVel, targetRPM);
        telemetry.addData("Current Velocity", "%.1f TPS (%.1f RPM)", currentVel, currentRPM);
        telemetry.addData("Error", "%.1f TPS", error);
        telemetry.addData("Top Flywheel Current", "%.2f A", topFlywheelCurrent);
        telemetry.addData("Bottom Flywheel Current", "%.2f A", bottomFlywheelCurrent);
        telemetry.addData("Motor Power", "%.3f", power);
        telemetry.addData("At Target", atTarget ? "Yupdeedoo" : "Uh oh spaghetti oh");
        telemetry.addLine();
        telemetry.addData("=== PID TUNING ===", "");
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kV", kV);
        telemetry.addData("Tolerance", velocityTolerance);

        dashboard.getTelemetry().addData("Flywheel Target RPM", targetRPM);
        dashboard.getTelemetry().addData("Flywheel Current RPM", currentRPM);
        dashboard.getTelemetry().addData("Flywheel Error", error);
        dashboard.getTelemetry().addData("Flywheel Power", power);

        telemetry.update();
        dashboard.getTelemetry().update();
    }

    public Command runToVelocity(double velocityTPS) {
        rebuildControlSystem();
        return new RunToVelocity(controlSystem, velocityTPS, velocityTolerance).requires(this);
    }

    public Command runToVelocity(double velocityTPS, double tolerance) {
        rebuildControlSystem();
        return new RunToVelocity(controlSystem, velocityTPS, tolerance).requires(this);
    }

    public Command runToRPM(double rpm) {
        return runToVelocity(rpmToTicks(rpm));
    }

    public Command runToRPM(double rpm, double tolerance) {
        return runToVelocity(rpmToTicks(rpm), tolerance);
    }

    public Command stop() {
        return runToVelocity(0.0);
    }

    public boolean atTargetVelocity() {
        KineticState tolerance = new KineticState(
                Double.POSITIVE_INFINITY,
                velocityTolerance,
                Double.POSITIVE_INFINITY
        );
        return controlSystem.isWithinTolerance(tolerance);
    }

    public double getCurrentVelocityTPS() {
        return motors.getState().getVelocity();
    }

    public double getCurrentRPM() {
        return ticksToRpm(getCurrentVelocityTPS());
    }

    private double rpmToTicks(double rpm) {
        return (rpm * ticksPerRev) / 60.0;
    }

    private double ticksToRpm(double ticks) {
        return (ticks * 60.0) / ticksPerRev;
    }
}