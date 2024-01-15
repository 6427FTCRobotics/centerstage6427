package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@TeleOp
@Config
@SuppressLint("DefaultLocale")
public final class TeleOP extends LinearOpMode {
    public static double leftMulti = 1.035;
    public static int hangPos = 1700;
    public static int finishHangPos = 700;
    public static int liftSpeed = 40;
    public static int maxHeight = 2250;
    private int liftPos = 0;
    OptimizedRobot robot;
    Servo flipperLeft, flipperRight, outtakeServo;
    CRServo spinnerServo;
    DcMotor leftSlide, rightSlide, airplane;

    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(new OptimizedController(gamepad1), new OptimizedController(gamepad2), telemetry, hardwareMap, new CenterStageControllerMapping());
        // Cursed but will start the robot in the flipped state
        robot.synchronousDelayGateFINISH("flipping");

        robot.initializeRoadRunner();
        robot.getInternalRR().setPoseEstimate(endPos);

        airplane = robot.getMotor("airplane", Direction.REVERSE);

        flipperLeft = robot.getServo("flipperLeft");
        flipperRight = robot.getServo("flipperRight");
        outtakeServo = robot.getServo("outtakeServo");
        flipperLeft.setPosition(flipperStoragePos);
        flipperRight.setPosition(1 - flipperStoragePos);
        outtakeServo.setPosition(outtakeStoragePos);
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
            handleAirplane();
        }
    }

    private void handleAirplane() {
        airplane.setPower(robot.getControl("airplane") ? 1 : 0);
    }

    private void handleIntake() {
        double intakeSpeed = -robot.getControlFloat("intake");
        spinnerServo.setPower(intakeSpeed);
    }

    private void handleAngles() {
        if (robot.getControl("toggleOuttake")) {
            robot.setToggle("fullDown", false);
            robot.synchronousDelayGateCLOSE("flipping");
        } else if (flipperLeft.getPosition() == flipperOuttakePos) {
            robot.synchronousDelayGateOPEN("flipping", getRuntime(), flipDelayMS / 1000.0);
        }
        boolean shouldUseOuttakePos = (robot.getControl("hangPart1") || robot.getControl("toggleOuttake")) && currentSlidePos() >= flipHeight;
        double targetFlipperPos = shouldUseOuttakePos ? flipperOuttakePos : robot.getControl("fullDown") ? flipperIntakePos : flipperStoragePos;
        flipperLeft.setPosition(targetFlipperPos);
        flipperRight.setPosition(1 - targetFlipperPos);
        outtakeServo.setPosition(shouldUseOuttakePos ? outtakeOuttakePos : robot.getControl("fullDown") ? outtakeIntakePos : outtakeStoragePos);
    }

    private void handleLift() {
        if (robot.getControl("hangPart1")) {
            liftPos = robot.getControl("hangPart2") ? finishHangPos : hangPos;
        } else {
            if (robot.getControl("toggleOuttake")) {
                liftPos = Math.min(Math.max(lowestOuttake + 150, liftPos - (int) (robot.getControlFloat("liftControl") * liftSpeed)), maxHeight);
            } else if (robot.synchronousDelayGateOPEN("flipping", getRuntime(), flipDelayMS / 1000.0)) {
                liftPos = 0;
            } else {
                liftPos = untilFlipPos;
            }
        }
        leftSlide.setTargetPosition((int) (liftPos * leftMulti));
        rightSlide.setTargetPosition(liftPos);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("left", leftSlide.getCurrentPosition());
        packet.put("right", rightSlide.getCurrentPosition());
        packet.put("target", liftPos);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }

    private void handleMovement() {
        if (robot.getControl("resetHeading")) {
            robot.getInternalRR().setPoseEstimate(new Pose2d(0, 0, 0));
        }
        Pose2d poseEstimate = robot.getInternalRR().getPoseEstimate();
        Vector2d input = new Vector2d(-gamepad1.left_stick_y * (robot.getControl("slow") ? 0.25 : 1), -gamepad1.left_stick_x * (robot.getControl("slow") ? 0.25 : 1));
        if (!robot.getControl("armaanMode")) {
            input = input.rotated(-poseEstimate.getHeading());
        }
        Pose2d output = new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * (robot.getControl("slow") ? 0.25 : 1));
        robot.getInternalRR().setWeightedDrivePower(output);

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

    private int targetSlidePos() {
        return (leftSlide.getTargetPosition() + rightSlide.getTargetPosition()) / 2;
    }
}
