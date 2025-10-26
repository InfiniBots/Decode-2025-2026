package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WheelTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                leftFront.setPower(1);
            }

            else if(gamepad1.b) {
                rightFront.setPower(1);
            }

            else if(gamepad1.x) {
                rightRear.setPower(1);
            }
            else if(gamepad1.y) {
                leftRear.setPower(1);
            }

            else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftRear.setPower(0);
            }

        }
    }
}
