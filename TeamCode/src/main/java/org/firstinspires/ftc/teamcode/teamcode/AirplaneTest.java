package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Config
public class AirplaneTest extends LinearOpMode {
    public static double airplaneSpeed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo1 = hardwareMap.crservo.get("airplane1");
        CRServo servo2 = hardwareMap.crservo.get("airplane2");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            servo1.setPower(airplaneSpeed);
            servo2.setPower(-airplaneSpeed);
        }
    }
}
