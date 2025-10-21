package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimelightDistance extends OpMode {

    private Limelight3A limelight;
    private double distance;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null) {


        }
        if (llResult != null && llResult.isValid()) {
            distance = distanceAprilTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
        }


    }

    public double distanceAprilTag(double ta) {
        double scale = 30692.95; // value requires fine tuning will do later
        double distance = Math.sqrt(scale/ta) + 2;
        return distance;
    }
}





