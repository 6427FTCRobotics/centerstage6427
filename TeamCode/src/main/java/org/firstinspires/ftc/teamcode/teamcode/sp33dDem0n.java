package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class sp33dDem0n extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Telemetry.Log log;
    int leftSpeed = 0;
    int rightSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = (DcMotor) hardwareMap.get("frontLeftMotor");
        frontRightMotor = (DcMotor) hardwareMap.get("frontRightMotor");
        backLeftMotor = (DcMotor) hardwareMap.get("backLeftMotor");
        backRightMotor = (DcMotor) hardwareMap.get("backRightMotor");
        log = telemetry.log();
        waitForStart();
        while (!isStopRequested()) {
            log.add("L: " + leftSpeed + " || R: " + rightSpeed);

            leftSpeed = (int)(gamepad1.left_stick_y * -100);
            rightSpeed = (int)(gamepad1.left_stick_y * -100);

            if (gamepad1.dpad_up) {

                leftSpeed = 100;
                rightSpeed = 100;
            } else if (gamepad1.dpad_down) {

                leftSpeed = -100;
                rightSpeed = -100;
            }

            if (gamepad1.right_stick_x > 0) rightSpeed -= (int)(gamepad1.right_stick_x * 100);
            else if (gamepad1.right_stick_x < 0) leftSpeed += (int)(gamepad1.right_stick_x * 100);

            frontLeftMotor.setPower(leftSpeed / 100d);
            backLeftMotor.setPower(leftSpeed / 100d);
            frontRightMotor.setPower(rightSpeed / 100d);
            backRightMotor.setPower(rightSpeed / 100d);

            sleep(10);
            log.clear();
        }
    }
}