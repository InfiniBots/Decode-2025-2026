package org.firstinspires.ftc.teamcode.subSystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.bylazar.configurables.annotations.Configurable;

@Autonomous
@Configurable
@SuppressWarnings("FieldCanBeLocal");
public class MotifLooking extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "All Parts");
        limelight.pipelineSwitch(0);

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) return;

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);

            int id = fiducial.getFiducialId();
            double x = fiducial.getTargetXDegrees();
            double y = fiducial.getTargetYDegrees();
            double distance = fiducial.getRobotPoseTargetSpace().getY();
            telemetry.addData("Fiducial" + id, "is" + distance + " meters away");
            telemetry.update();

    }

}
