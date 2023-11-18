package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import static org.firstinspires.ftc.teamcode.teamcode.Shared.*;
@Autonomous
public class BlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OptimizedRobot robot = new OptimizedRobot(telemetry, hardwareMap);
        robot.initializeRoadRunner();
        robot.getInternalRR().setPoseEstimate(new Pose2d(0, 0));
        waitForStart();
        robot.followRRTrajectory(robot.getInternalRR().trajectoryBuilder(new Pose2d()).strafeLeft(parkDist));
    }
}
