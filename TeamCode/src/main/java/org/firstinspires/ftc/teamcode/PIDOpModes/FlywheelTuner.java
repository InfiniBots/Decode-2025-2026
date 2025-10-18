package org.firstinspires.ftc.teamcode.PIDOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;
import static dev.nextftc.bindings.Bindings.*;

@Configurable
@Config
@TeleOp
public class FlywheelTuner extends NextFTCOpMode {
    private FlywheelVelocityPID flywheel;
    public static double targetRPM = 3000.0;

    @Override
    public void onInit() {
        flywheel = new FlywheelVelocityPID(hardwareMap, telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        button(() -> gamepad1.a).whenBecomesTrue(flywheel.runToRPM(targetRPM));
        button(() -> gamepad1.b).whenBecomesTrue(flywheel.stop());
    }

    @Override
    public void onUpdate() {
    }
}