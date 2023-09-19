package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;

@TeleOp
public class Test2 extends OpMode {
    OptimizedRobot robot;
    OptimizedController controller1, controller2;

    @Override
    public void init() {
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);
        robot = new OptimizedRobot(controller1, controller2, telemetry, hardwareMap, new FreightFrenzyControllerMapping());
//        robot.getInternalRR().setWeightedDrivePower();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            hardwareMap.dcMotor.get("frontRightMotor").setPower(1);
        }
        if (gamepad1.left_bumper) {
            hardwareMap.dcMotor.get("frontLeftMotor").setPower(1);
        }
        if (gamepad1.a) {
            hardwareMap.dcMotor.get("backLeftMotor").setPower(1);
        }
        if (gamepad1.b) {
            hardwareMap.dcMotor.get("backRightMotor").setPower(1);
        }
        robot.updateDrive(controller1, controller2, true, false, 1d, OptimizedRobot.RobotDirection.BACK, OptimizedRobot.RobotDirection.BACK, false);
    }
}
