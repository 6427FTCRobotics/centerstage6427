package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.FEET_PER_METER;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.lowestOuttake;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.targetX;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.targetZ;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internal.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

@Autonomous
@Config
public class RedAuto extends Auto {
    public static double strafeForLeftTape = 8.5;
    public static double backwardForLeftTape = 19.5;

    public static double strafeForRightTape = 12;
    public static double backwardForRightTape = 15;

    public static double strafeForMidTape = 20;

    public static double backwardForMidTape = 36.5;
    public static double backwardForHide = 5.5;
    public static double backwardForLeftBoard = 29.5;
    public static double backwardForMidBoard = 23.5;
    public static double backwardForRightBoard = 17.5;
    public static double strafeForBoardVision = 45;

    @Override
    public void runOpMode() throws InterruptedException {
        endPos = new Pose2d(0, 0, -Math.PI / 2);
        SampleMecanumDrive drive = beforeStart();

        TrajectorySequence startToRightTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForRightTape, strafeForRightTape, Math.PI))
                .build();
        TrajectorySequence rightTapeToBoard = drive.trajectorySequenceBuilder(startToRightTape.end())
                .back(fleeDist)
                .lineToLinearHeading(new Pose2d(-backwardForRightBoard, strafeForBoardVision, -Math.PI / 2))
                .build();
        TrajectorySequence rightBack = drive.trajectorySequenceBuilder(rightTapeToBoard.end())
                .lineToLinearHeading(new Pose2d(-backwardForHide, strafeForBoardVision - 15, -Math.PI / 2))
                .build();
        TrajectorySequence rightForward = drive.trajectorySequenceBuilder(rightBack.end())
                .lineToLinearHeading(new Pose2d(-backwardForHide, strafeForBoardVision, -Math.PI / 2))
                .build();

        TrajectorySequence startToLeftTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForLeftTape, strafeForRightTape, Math.PI))
                .lineToLinearHeading(new Pose2d(-backwardForLeftTape, -strafeForLeftTape, Math.PI))
                .build();
        TrajectorySequence leftTapeToBoard = drive.trajectorySequenceBuilder(startToLeftTape.end())
                .lineToLinearHeading(new Pose2d(-backwardForLeftBoard, strafeForBoardVision, -Math.PI / 2))
                .build();
        TrajectorySequence leftBack = drive.trajectorySequenceBuilder(leftTapeToBoard.end())
                .lineToLinearHeading(new Pose2d(-backwardForHide, strafeForBoardVision - 15, -Math.PI / 2))
                .build();
        TrajectorySequence leftForward = drive.trajectorySequenceBuilder(leftBack.end())
                .lineToLinearHeading(new Pose2d(-backwardForHide, strafeForBoardVision, -Math.PI / 2))
                .build();

        TrajectorySequence startToMidTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForMidTape, strafeForMidTape, -Math.PI / 2))
                .build();
        TrajectorySequence midTapeToBoard = drive.trajectorySequenceBuilder(startToMidTape.end())
                .back(fleeDist)
                .lineToLinearHeading(new Pose2d(-backwardForMidBoard, strafeForBoardVision, -Math.PI / 2))
                .build();
        TrajectorySequence midBack = drive.trajectorySequenceBuilder(midTapeToBoard.end())
                .lineToLinearHeading(new Pose2d(-backwardForHide, strafeForBoardVision - 15, -Math.PI / 2))
                .build();
        TrajectorySequence midForward = drive.trajectorySequenceBuilder(midBack.end())
                .lineToLinearHeading(new Pose2d(-backwardForHide, strafeForBoardVision, -Math.PI / 2))
                .build();

        pipeline = new Pipeline(Pipeline.Team.Red, Pipeline.Distance.Close);
        robot.initializeOpenCVPipeline(true, pipeline);
        waitForStart();
        waitForSpike();
        switch (pipeline.spikePos) {
            case Left:
                drive.followTrajectorySequence(startToLeftTape);
                break;
            case Right:
                drive.followTrajectorySequence(startToRightTape);
                break;
            case Center:
                drive.followTrajectorySequence(startToMidTape);
                break;
        }
        moveFlippers(flipperIntakePos);
        outtakeServo.setPosition(outtakeIntakePos);
        outtake();
        moveFlippers(flipperStoragePos);
        outtakeServo.setPosition(outtakeStoragePos);
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
        moveSlides(lowestOuttake);
        flipOut();
        outtakeLong();
        switch (pipeline.spikePos) {
            case Left:
                drive.followTrajectorySequence(leftBack);
                break;
            case Center:
                drive.followTrajectorySequence(midBack);
                break;
            case Right:
                drive.followTrajectorySequence(rightBack);
                break;
        }
        moveFlippers(flipperIntakePos);
        outtakeServo.setPosition(outtakeIntakePos);
        sleep(flipDelayMS);
        moveSlides(100);
        switch (pipeline.spikePos) {
            case Left:
                drive.followTrajectorySequence(leftForward);
                break;
            case Center:
                drive.followTrajectorySequence(midForward);
                break;
            case Right:
                drive.followTrajectorySequence(rightForward);
                break;
        }
    }
}
