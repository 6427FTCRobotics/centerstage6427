package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

public class DistanceSensorTest extends LinearOpMode {
    Rev2mDistanceSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "coneDistance");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Distance: ", sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
