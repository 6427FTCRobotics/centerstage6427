package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@Autonomous
public class SpikeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OptimizedRobot robot = new OptimizedRobot(telemetry, hardwareMap);
        Pipeline pipeline = new Pipeline(Pipeline.Team.Blue, Pipeline.Distance.Close);
        robot.initializeOpenCVPipeline(true, pipeline);
        waitForStart();
        while (pipeline.spikePos == null) {
            sleep(20);
        }
        while (!isStopRequested() && opModeIsActive()) {
            switch (pipeline.spikePos) {
                case Left:
                    telemetry.log().add("Left");
                    break;
                case Center:
                    telemetry.log().add("Center");
                    break;
                case Right:
                    telemetry.log().add("Right");
                    break;
            }
        }
    }
}
