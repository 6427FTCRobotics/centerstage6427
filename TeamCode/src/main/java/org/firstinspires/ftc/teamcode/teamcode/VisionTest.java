package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;

@Autonomous
@Config
public class VisionTest extends LinearOpMode {
    public static int targetTag = 5;

    OptimizedRobot robot;
    SampleMecanumDrive drive;
    Pipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);
        robot.initializeRoadRunner();
        drive = robot.getInternalRR();
        drive.setPoseEstimate(new Pose2d(0, 0, Math.PI / 2));
        pipeline = new Pipeline(Pipeline.Team.Red);
        robot.initializeOpenCVPipeline(true, pipeline);
        pipeline.finishSpike();
        waitForStart();
            Optional<AprilTagDetection> optionalDetection = Optional.empty();
            while (!isStopRequested() && !optionalDetection.isPresent()) {
                ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();
                if (detections != null) {
                    optionalDetection = detections.stream().filter(x -> x.id == targetTag).findFirst();
                }
                drive.update();
            }
            AprilTagDetection detection = optionalDetection.get();
            telemetry.addLine(String.format("X Miss = %.2f, Z Miss = %.2f", detection.pose.x * FEET_PER_METER * 12, detection.pose.z * FEET_PER_METER * 12 + targetZ));
            Pose2d currentPos = drive.getPoseEstimate();
            Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(currentPos).lineToLinearHeading(new Pose2d(currentPos.getY() - detection.pose.x * FEET_PER_METER * 12 + targetX, currentPos.getX() - detection.pose.z * FEET_PER_METER * 12 + targetZ, currentPos.getHeading() - Math.toRadians(rot.firstAngle))).build());
    }
}
