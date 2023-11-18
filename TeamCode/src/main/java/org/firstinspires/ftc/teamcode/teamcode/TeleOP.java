package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import kotlin.Metadata;
import kotlin.jvm.internal.Intrinsics;

import org.firstinspires.ftc.teamcode.internal.ControllerMapping;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@TeleOp
@Config
@SuppressLint("DefaultLocale")
public final class TeleOP extends LinearOpMode {
    public static int liftSpeed = 40;
    private int liftPos = 2000;
    OptimizedRobot robot;
    Servo flipperLeft, flipperRight, outtakeServo;
    CRServo spinnerServo;
    DcMotor leftSlide, rightSlide;

    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(new OptimizedController(gamepad1), new OptimizedController(gamepad2), telemetry, hardwareMap, new CenterStageControllerMapping());
        // Cursed but will start the robot in the flipped state
        robot.synchronousDelayGateFINISH("flipping");

        robot.initializeRoadRunner();
        robot.getInternalRR().setPoseEstimate(new Pose2d(0, 0));

        flipperLeft = robot.getServo("flipperLeft");
        flipperRight = robot.getServo("flipperRight");
        outtakeServo = robot.getServo("outtakeServo");
        flipperLeft.setPosition(intakeFlipperPos);
        flipperRight.setPosition(1 - intakeFlipperPos);
        outtakeServo.setPosition(outtakeIntakePos);
        leftSlide = robot.getMotor("leftSlide", RunMode.RUN_TO_POSITION, Direction.REVERSE);
        leftSlide.setPower(liftPower);
        rightSlide = robot.getMotor("rightSlide", RunMode.RUN_TO_POSITION);
        rightSlide.setPower(liftPower);
        spinnerServo = hardwareMap.crservo.get("spinnerServo");
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            handleIntake();
            handleAngles();
            handleLift();
            handleMovement();
        }
    }

    private void handleIntake() {
        double intakeSpeed = -robot.getControlFloat("intake");
        spinnerServo.setPower(intakeSpeed);
    }

    private void handleAngles() {
        if (robot.getControl("toggleOuttake")) {
            robot.synchronousDelayGateCLOSE("flipping");
        } else if (flipperLeft.getPosition() == outtakeFlipperPos) {
            robot.synchronousDelayGateOPEN("flipping", getRuntime(), 1.5);
        }
        boolean shouldUseOuttakePos = robot.getControl("toggleOuttake") && currentSlidePos() >= lowestOuttake - 50;
        double targetFlipperPos = shouldUseOuttakePos ? outtakeFlipperPos : intakeFlipperPos;
        flipperLeft.setPosition(targetFlipperPos);
        flipperRight.setPosition(1 - targetFlipperPos);
        outtakeServo.setPosition(shouldUseOuttakePos ? outtakeOuttakePos : outtakeIntakePos);
    }

    private void handleLift() {
        if (robot.getControl("toggleOuttake")) {
            liftPos = Math.max(lowestOuttake, liftPos - (int) (robot.getControlFloat("liftControl") * liftSpeed));
        } else if (robot.synchronousDelayGateOPEN("flipping", getRuntime(), 1.5)) {
            liftPos = robot.getControl("fullDown") ? 0 : 100;
        } else {
            liftPos = lowestOuttake;
        }
        leftSlide.setTargetPosition(liftPos);
        rightSlide.setTargetPosition(liftPos);
    }

    private void handleMovement() {
        robot.getInternalRR().setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y * (robot.getControl("slow") ? 0.25 : 1), -gamepad1.left_stick_x * (robot.getControl("slow") ? 0.25 : 1), -gamepad1.right_stick_x * (robot.getControl("slow") ? 0.25 : 1)));

        // Update everything. Odometry. Etc.
        robot.getInternalRR().update();

        // Print pose to telemetry
        telemetry.addData("x", robot.getRRPoseEstimate().getX());
        telemetry.addData("y", robot.getRRPoseEstimate().getY());
        telemetry.addData("heading", robot.getRRPoseEstimate().getHeading());
        telemetry.addData("liftPos", liftPos);
        telemetry.update();
    }

    private int currentSlidePos() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2;
    }
}
