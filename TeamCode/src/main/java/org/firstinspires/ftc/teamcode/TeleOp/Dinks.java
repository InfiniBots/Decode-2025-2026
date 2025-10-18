package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID;
import org.firstinspires.ftc.teamcode.subSystem.Intakooo;
import org.firstinspires.ftc.teamcode.subSystem.Turrrrrrrrrreeeeeeeeetttttttt;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.*;

@Config
@TeleOp
public class Dinks extends NextFTCOpMode {

    private FlywheelVelocityPID flywheel;
    private Intakooo intake;
    private Turrrrrrrrrreeeeeeeeetttttttt turret;
    public static double flywheelRPM = 3000.0;

    private final MotorEx frontLeftMotor = new MotorEx("leftFront").reversed();
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor = new MotorEx("leftRear").reversed();
    private final MotorEx backRightMotor = new MotorEx("rightRear");
    @Override
    public void onInit() {
        flywheel = new FlywheelVelocityPID(hardwareMap, telemetry);
        intake = new Intakooo(hardwareMap);
        turret = new Turrrrrrrrrreeeeeeeeetttttttt(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(turret.rotateClockwise(Turrrrrrrrrreeeeeeeeetttttttt.degrees45));

        button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(turret.rotateCounterClockwise(Turrrrrrrrrreeeeeeeeetttttttt.degrees45));

        button(() -> gamepad1.dpad_left)
                .whenBecomesTrue(turret.rotateClockwise(Turrrrrrrrrreeeeeeeeetttttttt.degrees90));

        button(() -> gamepad1.dpad_right)
                .whenBecomesTrue(turret.rotateCounterClockwise(Turrrrrrrrrreeeeeeeeetttttttt.degrees90));

        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(turret.center());

        button(() -> gamepad1.a)
                .whenBecomesTrue(flywheel.runToRPM(flywheelRPM));

        button(() -> gamepad1.b)
                .whenBecomesTrue(flywheel.stop());

        button(() -> gamepad1.right_trigger > 0.5)
                .whenBecomesTrue(intake.intake())
                .whenBecomesFalse(intake.stop());

        button(() -> gamepad1.left_trigger > 0.5)
                .whenBecomesTrue(intake.outtake())
                .whenBecomesFalse(intake.stop());
    }

    @Override
    public void onUpdate() {
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick: Turn");
        telemetry.addLine("L/R Bumper: Turret 45°");
        telemetry.addLine("Dpad L/R: Turret 90°");
        telemetry.addLine("A: Flywheel On | B: Flywheel Off");
        telemetry.addLine("RT: Intake | LT: Outtake");
    }
}