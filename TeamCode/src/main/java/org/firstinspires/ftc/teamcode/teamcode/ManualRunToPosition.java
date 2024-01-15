package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
@Config
public class ManualRunToPosition extends LinearOpMode {
    public static String name = "";
    public static int position = 0;
    public static double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            DcMotor motor = hardwareMap.dcMotor.get(name);
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }
}
