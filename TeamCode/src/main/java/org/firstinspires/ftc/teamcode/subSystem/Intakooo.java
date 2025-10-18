package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intakooo implements Subsystem {

    private final MotorEx intakeMotor;

    public Intakooo(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx("Intake");
    }

    @Override
    public void periodic() {
    }

    public Command intake() {
        return new InstantCommand(() -> intakeMotor.setPower(1.0)).requires(this);
    }

    public Command stop() {
        return new InstantCommand(() -> intakeMotor.setPower(0.0)).requires(this);
    }

    public Command outtake() {
        return new InstantCommand(() -> intakeMotor.setPower(-1.0)).requires(this);
    }
}