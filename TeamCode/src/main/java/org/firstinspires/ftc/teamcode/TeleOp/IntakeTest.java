package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {

    public static double power = 0.3;
    DcMotorEx Motor0;
    DcMotorEx Motor1;
    DcMotorEx Motor2;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        Motor0 = hardwareMap.get(DcMotorEx.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotorEx.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotorEx.class, "Motor2");

        //List<Double> timeStamps = new ArrayList<>();
        //List<Double> velocities = new ArrayList<>();
        List<String> log = new ArrayList<>();

        ElapsedTime timer = new ElapsedTime();
        double lastTime = 0;
        timer.reset();

        waitForStart();

        while (opModeIsActive()) {
            Motor0.setPower(1);
            Motor1.setPower(1);
            Motor2.setPower(1);

            double now = timer.seconds();
            double velocity = Motor2.getVelocity();

            if (now - lastTime >= 0.5) {
                log.add(String.format("t=%.2f  v=%.2f", now, velocity));
                if (log.size() > 20) {
                    log.remove(0);
                }

                for (String entry : log) {
                    telemetry.addLine(entry);
                }
                telemetry.update();
                lastTime = now;
            }
        }
    }
}
