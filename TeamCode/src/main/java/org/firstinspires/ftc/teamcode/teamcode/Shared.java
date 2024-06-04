package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Shared {
    public static double fleeDist = 12;
    public static double smallFleeDist = 2;
    public static double outtakePower = 1;
    public static int outtakeMs = 700;
    public static double flipperStoragePos = 0.1;
    public static double flipperIntakePos = 0.15; // +0.09
    public static double flipperOuttakePos = 0.6;
    public static double outtakeStoragePos = 1;
    public static double outtakeIntakePos = 0.75; // flipped
    public static double outtakeOuttakePos = 1;
    public static int lowestOuttake = 530;
    public static double flipHeight = 115;

    public static double liftPower = 1;
    public static double targetZ = 10;
    public static double targetX = -1.5;
    public static int flipDelayMS = 300;
    public static int untilFlipPos = 360;
    public static Pose2d endPos = new Pose2d();

    public static final double FEET_PER_METER = 3.28084;
}
