package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Optional;

@Autonomous
@Config
public class Auto extends LinearOpMode {
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
    public static int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public static float DECIMATION_HIGH = 3;
    public static float DECIMATION_LOW = 2;
    public static float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final double FEET_PER_METER = 3.28084;

    State state = State.waiting;

    Pipeline pipeline;
    OptimizedRobot robot;
    int numFramesWithoutDetection = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);
        robot.initializeRoadRunner();
        pipeline = new Pipeline(0.0508, fx, fy, cx, cy, Pipeline.Team.Blue);
        robot.initializeOpenCVPipeline(true, pipeline);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            switch (pipeline.spikePos) {
                case Left:
                    break;
                case Right:
                    break;
                case Center:
                    break;
                default:
                    break;
            }
        }
    }

    private void scanApril() {
        pipeline.finishSpike();
        ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if (detections != null) {
            telemetry.addData("FPS", robot.loader.phoneCam.getFps());
            telemetry.addData("Overhead ms", robot.loader.phoneCam.getOverheadTimeMs());
            telemetry.addData("Pipeline ms", robot.loader.phoneCam.getPipelineTimeMs());

            // If we don't see any tags
            if (detections.isEmpty()) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    pipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    pipeline.setDecimation(DECIMATION_HIGH);
                }

                telemetry.addLine(String.valueOf(state));

                for (AprilTagDetection detection : detections) {
                    Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                }
            }

            switch (state) {
                case waiting:
                    Optional<AprilTagDetection> optionalDetection = detections.stream().filter(x->x.id == 2).findFirst();
                    if (optionalDetection.isPresent()) {
                        AprilTagDetection detection = optionalDetection.get();
                        if (detection.pose.x < 0) {
                            robot.followRRTrajectory(robot.getInternalRR().trajectoryBuilder(robot.getRRPoseEstimate())
                                    .strafeRight(-detection.pose.x * FEET_PER_METER * 12));
                        } else {
                            robot.followRRTrajectory(robot.getInternalRR().trajectoryBuilder(robot.getRRPoseEstimate())
                                    .strafeLeft(detection.pose.x * FEET_PER_METER * 12));
                        }
                        state = State.centered;
                    }
            }

            telemetry.update();
        }

        sleep(20);
    }

    enum State {
        waiting, centered, scanning
    }
}
