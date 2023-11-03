package org.firstinspires.ftc.teamcode.teamcode;

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
    public static double intakeFlipperPos = 0.5;
    public static double outtakeIntakePos = 0.25;
    public static double outtakeOuttakePos = 0.2;

    // Start at lowestOuttake once known
    private int liftPos = 0;
    public static int fullIntakeThreshold = 300;
    public static double fullIntakePos = 0.4;
    public static double outtakeFlipperPos = 0;

    public static int liftSpeed = 40;

    public static int lowestOuttake = 2000;

    OptimizedRobot robot;
    Servo flipperLeft, flipperRight, outtakeServo;
    CRServo spinnerServo;
    DcMotor leftSlide, rightSlide, intakeMotor;

    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(new OptimizedController(gamepad1), new OptimizedController(gamepad2), telemetry, hardwareMap, new CenterStageControllerMapping());
        // Cursed but will start the robot in the flipped state
        robot.synchronousDelayGateOPEN("flipping", getRuntime(), 1);
        robot.initializeRoadRunner();
        robot.getInternalRR().setPoseEstimate(new Pose2d(0, 0));

        flipperLeft = robot.getServo("flipperLeft");
        flipperRight = robot.getServo("flipperRight");
        outtakeServo = robot.getServo("outtakeServo");
        flipperLeft.setPosition(fullIntakePos);
        flipperRight.setPosition(1 - fullIntakePos);
        outtakeServo.setPosition(outtakeIntakePos);
        leftSlide = robot.getMotor("leftSlide", RunMode.RUN_TO_POSITION, Direction.REVERSE);
        leftSlide.setPower(1.0);
        rightSlide = robot.getMotor("rightSlide", RunMode.RUN_TO_POSITION);
        rightSlide.setPower(1.0);
        spinnerServo = hardwareMap.crservo.get("spinnerServo");
        intakeMotor = robot.getMotor("intakeMotor", Direction.REVERSE);
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
        intakeMotor.setPower(intakeSpeed);
    }

    private void handleAngles() {
        if (robot.getControl("toggleOuttake")) {
            robot.synchronousDelayGateCLOSE("flipping");
        } else if (flipperLeft.getPosition() == outtakeFlipperPos) {
            robot.synchronousDelayGateOPEN("flipping", getRuntime(), 1);
        }
        boolean shouldUseOuttakePos = robot.getControl("toggleOuttake") && leftSlide.getCurrentPosition() >= lowestOuttake - 50;
        double targetFlipperPos = shouldUseOuttakePos ? outtakeFlipperPos : liftPos < fullIntakeThreshold ? fullIntakePos : intakeFlipperPos;
        flipperLeft.setPosition(targetFlipperPos);
        flipperRight.setPosition(1 - targetFlipperPos);
        outtakeServo.setPosition(shouldUseOuttakePos ? outtakeOuttakePos : outtakeIntakePos);
    }

    private void handleLift() {
        if (robot.getControl("toggleOuttake")) {
            liftPos = Math.min(lowestOuttake, liftPos - (int) (robot.getControlFloat("liftControl") * liftSpeed));
        } else if (robot.synchronousDelayGateOPEN("flipping", getRuntime(), 1)) {
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
}
