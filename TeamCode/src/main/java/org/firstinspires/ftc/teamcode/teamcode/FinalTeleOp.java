package org.firstinspires.ftc.teamcode.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.OptimizedController;
import org.firstinspires.ftc.teamcode.internal.OptimizedRobot;
import org.firstinspires.ftc.teamcode.internal.RobotConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name = "Final TeleOP")
public class FinalTeleOp extends OpMode {
    DcMotor duckSpinner, arm, odometry, intake;
    OptimizedRobot robot;
    OptimizedController controller1, controller2;
    ColorSensor colorSensor;
    Telemetry.Log log;
    int armStart = -3200;
    int frontHighOuttake = -2220;
    int backHighOuttake = -900;
    int frontLowOuttake = -2900;
    int backLowOuttake = -420;
    boolean detectingBlock = true;

    @Override
    public void init() {
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);

        log = telemetry.log();

        robot = new OptimizedRobot(controller1, controller2, telemetry, hardwareMap, new CenterStageControllerMapping());
        duckSpinner = robot.getMotor("duckSpinner");
        arm = robot.getMotor("arm", DcMotor.RunMode.RUN_TO_POSITION);
        odometry = robot.getMotor("odometry", DcMotor.RunMode.RUN_USING_ENCODER);
        intake = robot.getMotor("intake");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        // Duck Spinner
        if (robot.getControl("Duck")) {
            if (robot.synchronousDelayGateOPEN("Duck", getRuntime(), 0.8)) {
                duckSpinner.setPower(1);
            } else {
                duckSpinner.setPower(0.6);
            }
        } else if (robot.getControl("DuckReverse")) {
            if (robot.synchronousDelayGateOPEN("DuckReverse", getRuntime(), 0.8)) {
                duckSpinner.setPower(-1);
            } else {
                duckSpinner.setPower(-0.6);
            }
        } else {
            duckSpinner.setPower(0);
            robot.synchronousDelayGateCLOSE("Duck");
            robot.synchronousDelayGateCLOSE("DuckReverse");
        }

        // If Arm Is Being Manually Controlled
        if (robot.getControl("ArmControl")) {
            // Set Arm's Mode to Manual Control
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Set Power to "ArmSmooth" Float
            arm.setPower(Math.pow(robot.getControlFloat("ArmSmooth"), 3));
        } else {
            // Set Arm's Mode to Automatically Move To Position
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Set Arm's Power to full
            arm.setPower(RobotConfig.ARM_POWER);
            // If Arm is in Outtake Mode
            if (robot.getControl("ArmOuttake")) {
                // If Arm's Outtake Mode is the Front
                if (robot.getControl("ArmSide")) {
                    // Set to Front High Outtake if ArmHeight is true, otherwise to to Front Low Outtake
                    arm.setTargetPosition(robot.getControl("ArmHeight") ? frontLowOuttake : frontHighOuttake);
                } else {
                    // Set to Back High Outtake if ArmHeight is true, otherwise to to Back Low Outtake
                    arm.setTargetPosition(robot.getControl("ArmHeight") ? backLowOuttake : backHighOuttake);
                }
            } else {
                // Move Arm to Intake Position
                arm.setTargetPosition(armStart);
            }
        }

        // If IntakeButton is pressed
        if (robot.getControl("IntakeButton")) {
            // Intake with full power
            intake.setPower(1);
        } else if (robot.getControl("OuttakeButton")) {
            // Outtake with full power
            intake.setPower(-1);
        } else {
            // Set Intake/Outtake Power to Intake Float - Outtake Float
            intake.setPower(robot.getControlFloat("Intake") - robot.getControlFloat("Outtake"));
        }

        if (!detectingBlock && colorSensor.red() > 1000 && colorSensor.green() > 1000) {
            telemetry.speak("Box");
            detectingBlock = true;
        } else if (colorSensor.red() < 1000 || colorSensor.green() < 1000) {
            detectingBlock = false;
        }


//        log.add(colorSensor.red() + " " + colorSensor.green());
//        if (detectingBlock) {
//            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
//            telemetry.addData("TEST", "<h1>88888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888</h1>");
//        } else {
//            telemetry.clearAll();
//        }

        // Set odometry power to the left stick cubed, to add mild curving
        odometry.setPower(Math.pow(robot.getControlFloat("Odometry"), 3));
        log.add("Arm Position: " + arm.getCurrentPosition());
        log.clear();

        // Main drive method
        robot.updateDrive(controller1, controller2, true, false, 1d, OptimizedRobot.RobotDirection.BACK, OptimizedRobot.RobotDirection.BACK, false);
    }
}