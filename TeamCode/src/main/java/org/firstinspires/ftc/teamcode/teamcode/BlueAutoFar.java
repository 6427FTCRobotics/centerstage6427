package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internal.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class BlueAutoFar extends Auto {
    public static double strafeForLeftTape = 8.5;
    public static double backwardForLeftTape = 19.5;

    public static double strafeForRightTape = 12;
    public static double backwardForRightTape = 15;

    public static double strafeForMidTape = 20;
    public static double backwardForMidTape = 36.5;

    public static double strafeForCenter = 20;
    public static double backwardForCenter = 51;
    public static double toBoard = 104;

    @Override
    public void runOpMode() throws InterruptedException {
        endPos = new Pose2d(0, 0, Math.PI / 2);
        SampleMecanumDrive drive = beforeStart();

        TrajectorySequence startToRightTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForRightTape, strafeForRightTape, Math.PI))
                .build();
        TrajectorySequence rightTapeToCenter = drive.trajectorySequenceBuilder(startToRightTape.end())
                .forward(fleeDist * 2)
                .lineToLinearHeading(new Pose2d(-backwardForCenter, strafeForCenter, Math.PI / 2))
                .back(toBoard)
                .build();

        TrajectorySequence startToLeftTape = drive.trajectorySequenceBuilder(startToRightTape.end())
                .lineToLinearHeading(new Pose2d(-backwardForLeftTape, strafeForRightTape, Math.PI))
                .lineToLinearHeading(new Pose2d(-backwardForLeftTape, -strafeForLeftTape, Math.PI))
                .build();
        TrajectorySequence leftTapeToCenter = drive.trajectorySequenceBuilder(startToLeftTape.end())
                .back(smallFleeDist)
                .strafeRight(fleeDist * 2)
                .lineToLinearHeading(new Pose2d(-backwardForCenter, strafeForCenter, Math.PI / 2))
                .back(toBoard)
                .build();

        TrajectorySequence startToMidTape = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(new Pose2d(-backwardForMidTape, strafeForMidTape, -Math.PI / 2))
                .build();
        TrajectorySequence midTapeToCenter = drive.trajectorySequenceBuilder(startToMidTape.end())
                .back(fleeDist)
                .lineToLinearHeading(new Pose2d(-backwardForCenter, strafeForCenter, Math.PI / 2))
                .back(toBoard)
                .build();

        pipeline = new Pipeline(Pipeline.Team.Blue, Pipeline.Distance.Far);
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
