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
    //0.8, 0.2 intake
    // 0.3, 0 outtake
    public static double intakeFlipperPos = 0.5;
    public static double outtakeIntakePos = 0.25;
    public static double outtakeOuttakePos = 0.3;
    private int liftPos = 0;
    public static int fullIntakeThreshold = 300;
    public static double fullIntakePos = 0.4;
    public static double outtakeFlipperPos = 0.3;

    public static int liftSpeed = 20;


    public void runOpMode() throws InterruptedException {
        OptimizedRobot robot = new OptimizedRobot(new OptimizedController(gamepad1), new OptimizedController(gamepad2), telemetry, hardwareMap, new CenterStageControllerMapping());
        robot.initializeRoadRunner();
        robot.getInternalRR().setPoseEstimate(new Pose2d(0, 0));

        Servo flipperLeft = robot.getServo("flipperLeft");
        Servo flipperRight = robot.getServo("flipperRight");
        Servo outtakeServo = robot.getServo("outtakeServo");
        flipperLeft.setPosition(fullIntakePos);
        flipperRight.setPosition(1 - fullIntakePos);
        outtakeServo.setPosition(outtakeIntakePos);
        DcMotor leftSlide = robot.getMotor("leftSlide", RunMode.RUN_TO_POSITION, Direction.REVERSE);
        leftSlide.setPower(1.0);
        DcMotor rightSlide = robot.getMotor("rightSlide", RunMode.RUN_TO_POSITION);
        rightSlide.setPower(1.0);
        CRServo spinnerServo = hardwareMap.crservo.get("spinnerServo");
        DcMotor intakeMotor = robot.getMotor("intakeMotor", Direction.REVERSE);
        waitForStart();

        while (!this.isStopRequested() && this.opModeIsActive()) {
            double intakeSpeed = -robot.getControlFloat("intake");
            liftPos -= (int) (robot.getControlFloat("liftControl") * liftSpeed);
            double targetFlipperPos = robot.getControl("toggleIntake") ? outtakeFlipperPos : liftPos < fullIntakeThreshold ? fullIntakePos : intakeFlipperPos;
            flipperLeft.setPosition(targetFlipperPos);
            flipperRight.setPosition(1 - targetFlipperPos);
            outtakeServo.setPosition(robot.getControl("toggleIntake") ? outtakeOuttakePos : outtakeIntakePos);
            spinnerServo.setPower(intakeSpeed);
            intakeMotor.setPower(intakeSpeed);
            leftSlide.setTargetPosition(liftPos);
            rightSlide.setTargetPosition(liftPos);
            robot.getInternalRR().setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

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
}
