package org.firstinspires.ftc.teamcode.subSystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Turrrrrrrrrreeeeeeeeetttttttt implements Subsystem {

    private final MotorEx turretMotor;
    public static double degrees90 = 90.0;
    public static double degrees45 = 45.0;
    public static double ticksPerDegree = 384.5 / 360.0;
    public static double motorPower = 0.5;
    private double currentTargetDegrees = 0.0;

    public Turrrrrrrrrreeeeeeeeetttttttt(HardwareMap hardwareMap) {
        turretMotor = new MotorEx("Turret").zeroed();
        turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setCurrentPosition(0);
        turretMotor.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
    }

    public Command rotateClockwise(double degrees) {
        return new InstantCommand(() -> {
            currentTargetDegrees += degrees;
            int targetTicks = (int)(currentTargetDegrees * ticksPerDegree);
            turretMotor.getMotor().setTargetPosition(targetTicks);
            turretMotor.getMotor().setPower(motorPower);
        }).requires(this);
    }

    public Command rotateCounterClockwise(double degrees) {
        return new InstantCommand(() -> {
            currentTargetDegrees -= degrees;
            int targetTicks = (int)(currentTargetDegrees * ticksPerDegree);
            turretMotor.getMotor().setTargetPosition(targetTicks);
            turretMotor.getMotor().setPower(motorPower);
        }).requires(this);
    }

    public Command goToPosition(double degrees) {
        return new InstantCommand(() -> {
            currentTargetDegrees = degrees;
            int targetTicks = (int)(currentTargetDegrees * ticksPerDegree);
            turretMotor.getMotor().setTargetPosition(targetTicks);
            turretMotor.getMotor().setPower(motorPower);
        }).requires(this);
    }

    public Command center() {
        return goToPosition(0.0);
    }
}