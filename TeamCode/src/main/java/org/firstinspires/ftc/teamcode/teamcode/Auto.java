package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

@Config
public abstract class Auto extends LinearOpMode {
    OptimizedRobot robot;
    Servo flipperLeft, flipperRight, outtakeServo;
    DcMotor leftSlide, rightSlide;
    CRServo spinnerServo;
    Pipeline pipeline;
    Pose2d startPos = new Pose2d(0, 0);

    protected SampleMecanumDrive beforeStart() {
        robot = new OptimizedRobot(telemetry, hardwareMap);
        robot.initializeRoadRunner();
        SampleMecanumDrive drive = robot.getInternalRR();
        drive.setPoseEstimate(startPos);

        flipperLeft = robot.getServo("flipperLeft");

        flipperRight = robot.getServo("flipperRight");

        outtakeServo = robot.getServo("outtakeServo");
        moveFlippers(flipperStoragePos);
        outtakeServo.setPosition(outtakeStoragePos);

        leftSlide = robot.getMotor("leftSlide", DcMotor.RunMode.RUN_TO_POSITION, DcMotorSimple.Direction.REVERSE);
        leftSlide.setPower(liftPower);

        rightSlide = robot.getMotor("rightSlide", DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(liftPower);
        spinnerServo = hardwareMap.crservo.get("spinnerServo");
        return drive;
    }

    protected void moveFlippers(double pos) {
        flipperLeft.setPosition(pos);
        flipperRight.setPosition(1 - pos);
    }

    protected void waitForSpike() {
        while (pipeline.spikePos == null) {
            sleep(20);
        }
        pipeline.finishSpike();
    }

    protected void outtake() {
        spinnerServo.setPower(outtakePower);
        sleep(outtakeMs);
        spinnerServo.setPower(0);
    }

    protected void outtakeLong() {
        spinnerServo.setPower(outtakePower);
        sleep(outtakeMs * 2L);
        spinnerServo.setPower(0);
    }

    protected void flipOut() {
        flipperLeft.setPosition(flipperOuttakePos);
        flipperRight.setPosition(1 - flipperOuttakePos);
        outtakeServo.setPosition(outtakeOuttakePos);
        sleep(flipDelayMS);
    }

    protected void flipIn() {
        flipperLeft.setPosition(flipperIntakePos);
        flipperRight.setPosition(1 - flipperIntakePos);
        outtakeServo.setPosition(outtakeIntakePos);
        sleep(flipDelayMS);
    }

    protected void moveSlides(int target) {
        leftSlide.setTargetPosition(target);
        rightSlide.setTargetPosition(target);
        while (!isStopRequested() && Math.abs(currentSlidePos() - target) > 50) {
            sleep(50);
        }
    }

    protected int currentSlidePos() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }

    protected AprilTagDetection getApril(int targetTag) {
        Optional<AprilTagDetection> optionalDetection = Optional.empty();
        while (!isStopRequested() && !optionalDetection.isPresent()) {
            ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();
            if (detections != null) {
                optionalDetection = detections.stream().filter(x -> x.id == targetTag).findFirst();
            }
        }
        AprilTagDetection detection = optionalDetection.get();
        telemetry.addLine(
                String.format("X Miss = %.2f, Z Miss = %.2f",
                        detection.pose.x * FEET_PER_METER * 12,
                        detection.pose.z * FEET_PER_METER * 12 + targetZ));
        return detection;
    }
}
