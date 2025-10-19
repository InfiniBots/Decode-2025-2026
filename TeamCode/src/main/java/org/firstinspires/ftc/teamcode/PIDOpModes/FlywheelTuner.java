package org.firstinspires.ftc.teamcode.PIDOpModes;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID;

import dev.nextftc.bindings.Bindings;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp
public class FlywheelTuner extends NextFTCOpMode {
    private FlywheelVelocityPID flywheel;
    public static double targetRPM = 3000.0;

    @Override
    public void onInit() {
        flywheel = new FlywheelVelocityPID(hardwareMap, telemetry);
        addComponents(
                new SubsystemComponent(flywheel),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        button(() -> gamepad1.a).whenBecomesTrue(flywheel.runToRPM(targetRPM));
        button(() -> gamepad1.b).whenBecomesTrue(flywheel.runToRPM(targetRPM));
    }

    @Override
    public void onUpdate() {
    }
}