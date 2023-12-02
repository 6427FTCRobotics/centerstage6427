package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.FEET_PER_METER;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.flipDelayMS;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.intakeFlipperPos;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.liftPower;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.lowestOuttake;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.outtakeFlipperPos;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.outtakeIntakePos;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.outtakeOuttakePos;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.targetX;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.targetZ;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internal.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

import java.util.ArrayList;
import java.util.Optional;

@Autonomous
@Config
public class BlueAuto extends Auto {
    public static double strafeForLeftTape = 12;
    public static double backwardForSideTape = 18;
    public static double backwardForMidTape = 34.5;
    public static double strafeForMidTape = 20;
    public static double strafeForRightTape = 11.5;
    public static double backwardForBoardVision = 24;
    public static double strafeForBoardVision = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = beforeStart();
        TrajectorySequence startToLeftTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForSideTape, -strafeForLeftTape, Math.PI))
                .build();
        TrajectorySequence leftTapeToRightTape = drive.trajectorySequenceBuilder(startToLeftTape.end())
                .lineToLinearHeading(new Pose2d(-backwardForSideTape, strafeForRightTape, Math.PI))
                .build();
        TrajectorySequence leftTapeToBoard = drive.trajectorySequenceBuilder(startToLeftTape.end())
                .back(fleeDist)
                .lineToLinearHeading(new Pose2d(-backwardForBoardVision, -strafeForBoardVision, Math.PI / 2))
                .build();
        TrajectorySequence rightTapeToBoard = drive.trajectorySequenceBuilder(leftTapeToRightTape.end())
                .lineToLinearHeading(new Pose2d(-backwardForBoardVision, -strafeForBoardVision, Math.PI / 2))
                .build();
        TrajectorySequence startToMidTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForMidTape, -strafeForMidTape, Math.PI / 2))
                .build();
        TrajectorySequence midTapeToBoard = drive.trajectorySequenceBuilder(startToMidTape.end())
                .lineToLinearHeading(new Pose2d(-backwardForBoardVision, -strafeForBoardVision, Math.PI / 2))
                .build();

        pipeline = new Pipeline(Pipeline.Team.Blue);
        robot.initializeOpenCVPipeline(true, pipeline);
        waitForStart();
        waitForSpike();
        switch (pipeline.spikePos) {
            case Left:
                drive.followTrajectorySequence(startToLeftTape);
                break;
            case Center:
                drive.followTrajectorySequence(startToMidTape);
                break;
            case Right:
                drive.followTrajectorySequence(startToLeftTape);
                drive.followTrajectorySequence(leftTapeToRightTape);
                break;
        }
        outtake();
        switch (pipeline.spikePos) {
            case Left:
                drive.followTrajectorySequence(leftTapeToBoard);
                break;
            case Center:
                drive.followTrajectorySequence(midTapeToBoard);
                break;
            case Right:
                drive.followTrajectorySequence(rightTapeToBoard);
                break;
        }
        pipeline.startApril();
        AprilTagDetection detection = getApril(pipeline.spikePos.targetTag());
        Pose2d currentPos = drive.getPoseEstimate();
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ,
                AngleUnit.DEGREES);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(currentPos)
                .lineToLinearHeading(
                        new Pose2d(
                                currentPos.getX() - detection.pose.x * FEET_PER_METER * 12 - targetX,
                                currentPos.getY() - detection.pose.z * FEET_PER_METER * 12 + targetZ,
                                currentPos.getHeading() - Math.toRadians(rot.firstAngle)))
                .build());
        moveSlides(lowestOuttake);
        flipOut();
        outtakeLong();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).forward(15).build());
        flipIn();
        moveSlides(500);
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(15).build());
    }
}
