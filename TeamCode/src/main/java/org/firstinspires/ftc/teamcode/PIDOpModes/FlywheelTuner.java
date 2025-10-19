package org.firstinspires.ftc.teamcode.PIDOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subSystem.FlywheelVelocityPID;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;
import static dev.nextftc.bindings.Bindings.*;

@Configurable
@Config
@TeleOp
public class FlywheelTuner extends OpMode {
    private FlywheelVelocityPID flywheel;
    public static double targetVelocity = 2000.0;

    @Override
    public void init() {
        flywheel = new FlywheelVelocityPID(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        flywheel.runToVelocity(targetVelocity);

    }
}