package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous
@Config
public class ManualServo extends LinearOpMode {

    // grabber 0.42 = closed, 0.37 = open
    // plane 0.5 closed 0.7 open
    public static double leftPos = 0;
    public static double outtakePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo flipperLeft = hardwareMap.servo.get("flipperLeft");
        Servo flipperRight = hardwareMap.servo.get("flipperRight");
        Servo outtakeServo = hardwareMap.servo.get("outtakeServo");
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            flipperLeft.setPosition(leftPos);
            flipperRight.setPosition(1 - leftPos);
            outtakeServo.setPosition(outtakePos);
        }
    }
}
