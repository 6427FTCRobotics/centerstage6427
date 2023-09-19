package org.firstinspires.ftc.teamcode.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class SolomanJeremy  extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    int speed = 0;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        OptimizedController controller1 = new OptimizedController(gamepad1);
        OptimizedController controller2 = new OptimizedController(gamepad2);
        OptimizedRobot robot = new OptimizedRobot(controller1, controller2, telemetry, hardwareMap, new FreightFrenzyControllerMapping());
        waitForStart();
        while (!isStopRequested()) {

            robot.updateDrive(controller1, controller2, true, false, 1d, OptimizedRobot.RobotDirection.BACK, OptimizedRobot.RobotDirection.BACK, false);
        }
    }
}
