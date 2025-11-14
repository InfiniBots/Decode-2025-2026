package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(group = "Test")
public class Drive2 extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;


    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        double x = -gamepad1.left_stick_x * 1.1;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeft = power * cos / max + turn;
        double frontRight = power * sin / max - turn;
        double backLeft = power * sin / max + turn;
        double backRight = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            frontLeft /= power + Math.abs(turn);
            frontRight /= power + Math.abs(turn);
            backLeft /= power + Math.abs(turn);
            backRight /= power + Math.abs(turn);
        }

        frontLeftMotor.setPower(frontLeft/-1);
        backLeftMotor.setPower(backLeft/-1);
        frontRightMotor.setPower(frontRight/1);
        backRightMotor.setPower(backRight/1);

    }
}

