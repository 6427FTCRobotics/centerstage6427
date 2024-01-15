package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class SlidesMover extends LinearOpMode {
    public static double leftMulti = 1;
    // 2400 max
    public static int targetPos = 0;
    public static double power = 1;
    public static double leftPos = 0.2;
    public static double outtakePos = 0;
    public static double intakeServoSpeed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo flipperLeft = hardwareMap.servo.get("flipperLeft");
        Servo flipperRight = hardwareMap.servo.get("flipperRight");
        Servo outtakeServo = hardwareMap.servo.get("outtakeServo");
        flipperLeft.setPosition(leftPos);
        flipperRight.setPosition(1 - leftPos);
        outtakeServo.setPosition(outtakePos);
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setTargetPosition(0);
        leftSlide.setPower(power);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");
        rightSlide.setTargetPosition(0);
        rightSlide.setPower(power);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CRServo spinnerServo = hardwareMap.crservo.get("spinnerServo");
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            flipperLeft.setPosition(leftPos);
            flipperRight.setPosition(1 - leftPos);
            outtakeServo.setPosition(outtakePos);
            leftSlide.setTargetPosition((int) (targetPos * leftMulti));
            rightSlide.setTargetPosition(targetPos);
            spinnerServo.setPower(intakeServoSpeed);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("left", leftSlide.getCurrentPosition());
            packet.put("right", rightSlide.getCurrentPosition());
            packet.put("target", targetPos);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
