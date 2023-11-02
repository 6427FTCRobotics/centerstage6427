package org.firstinspires.ftc.teamcode.internal.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.internal.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODOMETRY_TICKS_PER_REV;
    public static double WHEEL_RADIUS = org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODOMETRY_WHEEL_RADIUS; // in
    public static double GEAR_RATIO = org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODOMETRY_GEAR_RATIO; // output (wheel) speed / input (encoder) speed

    // seven and a half out of thirteen and one quarter
    // one and a quarter

    public static double PARALLEL_X = 0.875; // X is the up and down direction
    public static double PARALLEL_Y = 3.5; // Y is the strafe direction

    public static double PERPENDICULAR_X = -1.75;
    public static double PERPENDICULAR_Y = 0;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private final Encoder parallelEncoder;
    private final Encoder perpendicularEncoder;

    private final org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRightMotor"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeftMotor"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODO_X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODO_Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODO_X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * org.firstinspires.ftc.teamcode.internal.roadrunner.drive.DriveConstants.ODO_Y_MULTIPLIER
        );
    }
}