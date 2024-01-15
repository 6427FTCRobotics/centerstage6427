package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internal.roadrunner.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

@Autonomous
@Config
public class RedAutoFar extends Auto {
    public static double strafeForLeftTape = 15;
    public static double backwardForLeftTape = 15;

    public static double strafeForRightTape = 9;
    public static double backwardForRightTape = 19.5;

    public static double strafeForMidTape = 20;
    public static double backwardForMidTape = 38;

    public static double strafeForCenter = 20;
    public static double backwardForCenter = 50;
    public static double toBoard = 96;

    @Override
    public void runOpMode() throws InterruptedException {
        endPos = new Pose2d(0, 0, -Math.PI / 2);
        SampleMecanumDrive drive = beforeStart();

        TrajectorySequence startToLeftTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForLeftTape, -strafeForLeftTape, Math.PI))
                .build();
        TrajectorySequence leftTapeToCenter = drive.trajectorySequenceBuilder(startToLeftTape.end())
                .forward(fleeDist * 2)
                .lineToLinearHeading(new Pose2d(-backwardForCenter, -strafeForCenter, Math.PI / 2))
                .forward(toBoard)
                .build();

        TrajectorySequence startToRightTape = drive.trajectorySequenceBuilder(startToLeftTape.end())
                .lineToLinearHeading(new Pose2d(-backwardForRightTape, -strafeForLeftTape, Math.PI))
                .lineToLinearHeading(new Pose2d(-backwardForRightTape, strafeForRightTape, Math.PI))
                .build();
        TrajectorySequence rightTapeToCenter = drive.trajectorySequenceBuilder(startToRightTape.end())
                .back(smallFleeDist)
                .strafeLeft(fleeDist * 2)
                .lineToLinearHeading(new Pose2d(-backwardForCenter, -strafeForCenter, Math.PI / 2))
                .forward(toBoard)
                .build();

        TrajectorySequence startToMidTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForMidTape, -strafeForMidTape, Math.PI / 2))
                .build();
        TrajectorySequence midTapeToCenter = drive.trajectorySequenceBuilder(startToMidTape.end())
                .back(fleeDist)
                .lineToLinearHeading(new Pose2d(-backwardForCenter, -strafeForCenter, Math.PI / 2))
                .forward(toBoard)
                .build();

        pipeline = new Pipeline(Pipeline.Team.Red, Pipeline.Distance.Far);
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
                drive.followTrajectorySequence(startToRightTape);
                break;
        }
        moveFlippers(flipperIntakePos);
        outtakeServo.setPosition(outtakeIntakePos);
        outtake();
        moveFlippers(flipperStoragePos);
        outtakeServo.setPosition(outtakeStoragePos);
        switch (pipeline.spikePos) {
            case Left:
                drive.followTrajectorySequence(leftTapeToCenter);
                break;
            case Right:
                drive.followTrajectorySequence(rightTapeToCenter);
                break;
            case Center:
                drive.followTrajectorySequence(midTapeToCenter);
                break;
        }
    }
}
