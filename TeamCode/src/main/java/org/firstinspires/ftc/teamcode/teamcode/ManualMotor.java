package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous
public class ManualMotor extends LinearOpMode {
    public static double power = 0;
    public static String[] motors = {"frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor"};

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            for (String name: motors) {
                DcMotor motor = hardwareMap.dcMotor.get(name);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(power);
            }
        }
    }
}
