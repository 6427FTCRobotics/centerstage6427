package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Config
public class ManualCRServo extends LinearOpMode {
    public static double speed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get("spinnerServo");
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            servo.setPower(speed);
        }
    }
}
