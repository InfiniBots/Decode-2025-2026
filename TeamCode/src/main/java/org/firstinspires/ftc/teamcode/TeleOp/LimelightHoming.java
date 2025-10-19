package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

@TeleOp
public class LimelightHoming extends LinearOpMode {

    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    private final double kP = 0.0; // tune
    private final double kI = 0.0; // tune
    private final double kD = 0.0; // tune

    private double integral = 0;
    private double lastError = 0;

    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);

                double x = fiducial.getTargetXDegrees();

                double error = x;

                integral += error;
                double derivative = error - lastError;
                lastError = error;

                double newPower = kP * error + kI * integral + kD * derivative;

                turretMotor.setPower(newPower);

                telemetry.addData("Tracking ID: ", fiducial);
                telemetry.addData("Target X: ", x); 
                telemetry.addData("Motor Power: ", newPower);
                telemetry.update();


            }
        }

    }
}
