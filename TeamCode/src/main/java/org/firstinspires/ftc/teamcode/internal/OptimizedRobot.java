package org.firstinspires.ftc.teamcode.internal;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internal.roadrunner.util.Encoder;

import java.util.HashMap;
import java.util.List;

/**
 * This class is a "repackaged" version
 * of standard FTC Programming using OpMode/LinearOpMode classes.
 * Not only does it strive to help beginner programmers, but it also adds optimizations in the controllers and other things.
 *
 * @author Owen Boseley - Class of 2021
 */
@Repackaged
public class OptimizedRobot {

    /**
     * Robot direction state for drive train algorithm
     */
    public enum RobotDirection {
        FRONT, BACK, LEFT, RIGHT
    }

    /**
     * The current drive mode of the robot. More info below here: {@link #getDriveMode()}
     */
    private DriveMode driveMode = DriveMode.STOPPED;

    /**
     * The drive functions class mentioned below
     */
    private final OptimizedDriveFunctions functions;

    /**
     * Stores the telemetry instance from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * Used for outputting data to the drive station console.
     */
    private final Telemetry telemetry;

    /**
     * Stores the {@link OptimizedController}
     * Used for robot.getControl
     */
    private OptimizedController controller1 = null;

    private OptimizedController controller2 = null;

    /**
     * Stores an instance of our OpenCV Loader, which should never need to be edited unless for version changes
     */
    public OpenCVLoader loader = null;

    // Ignore these, these are internal stuff
    private boolean hasInitializedMotors = false;
    private boolean hasUpdatedDrive = false;

    /**
     * Holds the info for custom delays
     */
    private final HashMap<String, Boolean> delayInfoBools = new HashMap<>();
    private final HashMap<String, Double> delayInfoTimes = new HashMap<>();

    /**
     * Stores 4 booleans for if any of the drive train motors are disabled for testing purposes
     */
    private boolean[] disabledMotors = {false, false, false, false};


    /**
     * Our map of aliases of hardware components
     */
    private HashMap<String, List<String>> aliasMap = null;

    /**
     * This holds a custom set of controls for Teleop to use
     */
    @Experimental
    private HashMap<String, ControllerMapping.ControlInput> controlMap = null;

    /**
     * Stores the current robot status
     */
    private RobotStatus status;


    /**
     * Our mecanum drive for use in roadrunner
     */
    private SampleMecanumDrive mecanumDrive = null;


    /**
     * An array to store the drive train motors. Stored in the format: FL, FR, BL, BR.
     * P.S. All of the standards in this code are written out here: {@link RobotConfig}
     */
    protected DcMotor[] motors = new DcMotor[4];

    /**
     * Stores the hardwareMap from the phone. This is where we get all of our hardware components for usage in OpModes
     */
    private final HardwareMap internalMap;

    // Just some conversion ratios, ignore these
    private final double CM_PER_INCH = 2.56;
    private final double CM_PER_FOOT = 30.48;

    /**
     * The Wheel Radius of the main drive train wheels in CM
     */
    private final double WHEEL_RADIUS = RobotConfig.WHEEL_RADIUS * CM_PER_INCH;

    /**
     * The circumference of the drive train wheels in CM
     */
    protected final double CIRCUMFERENCE = Math.PI * WHEEL_RADIUS * 2;

    /**
     * An Enum representing the possible robot statuses
     */
    private enum RobotStatus {
        INITIALIZING, READY, IDLE, STOPPED, DRIVING
    }

    /**
     * An Enum representing the possible drive modes
     */
    public enum DriveMode {
        OMNI, TANK, STOPPED
    }

    /**
     * Constructor oi this class
     *
     * @param telemetry       The telemetry var inherited from OpMode/LinearOpMode
     * @param hardwareMap     The hardwareMap var inherited from OpMode/LinearOpMode
     */
    public OptimizedRobot(Telemetry telemetry, HardwareMap hardwareMap) {
        functions = new OptimizedDriveFunctions(this);

        internalMap = hardwareMap;
        this.telemetry = telemetry;

        status = RobotStatus.READY;
    }

    /**
     * Constructor of this class
     *
     * @param controller1     The first {@link OptimizedController} instance for teleop
     * @param controller2     The second {@link OptimizedController} instance for teleop
     * @param telemetry       The telemetry var inherited from OpMode/LinearOpMode
     * @param hardwareMap     The hardwareMap var inherited from OpMode/LinearOpMode
     * @param controlMap      A hashmap of string names to keys for teleop -- might be useless, idk
     */
    public OptimizedRobot(OptimizedController controller1, OptimizedController controller2, Telemetry telemetry, HardwareMap hardwareMap, ControllerMapping controlMap) {
        functions = new OptimizedDriveFunctions(this);
        this.controlMap = controlMap.initializeMapping(new HashMap<>());

        internalMap = hardwareMap;
        this.telemetry = telemetry;
        this.controller1 = controller1;
        this.controller2 = controller2;

        status = RobotStatus.READY;
    }

    /**
     * Used to initialize OpenCV and the camera for usage
     *
     * @param activateCam Whether or not to display the camera output on the phone
     * @usage Only call this method during initialization (I mean, you can call it after but like... why?)
     */
    public void initializeOpenCVPipeline(boolean activateCam, OptimizedOpenCVPipeline pipeline) {
        status = RobotStatus.INITIALIZING;
        loader = new OpenCVLoader(internalMap, activateCam, pipeline);

        status = RobotStatus.READY;
    }

    /**
     * Initializes our roadrunner shtuff
     */
    public void initializeRoadRunner() {
        mecanumDrive = new SampleMecanumDrive(internalMap);
    }

    /**
     * Used if you do NOT want to use the repackaged methods in this class -- up to you
     *
     * @return The mecanum drive obj used for RR processes
     */
    public SampleMecanumDrive getInternalRR() {
        return mecanumDrive;
    }

    /**
     * Returns position estimate for road runner
     *
     * @return The current RoadRunner Position Estimate
     */
    @Repackaged
    public Pose2d getRRPoseEstimate() {
        return mecanumDrive.getPoseEstimate();
    }

    /**
     * Used to follow a given trajectory using RR
     *
     * @param trajectory the trajectory BUILDER (without the .build() at the end) to follow
     */
    @Repackaged
    public void followRRTrajectory(TrajectoryBuilder trajectory) {
        mecanumDrive.followTrajectory(trajectory.build());
    }

    /**
     * Useful delay method for teleop opmodes -- want to delay a piece of code without stopping the thread entirely? Use THIS!
     * NOTE: This method will only OPEN the gate after the delay, and will not close it. You'll need to call {@link #synchronousDelayGateCLOSE(String)} to close it!
     * NOTE: Use
     *
     * @param delayName      The delay name (can be anything you want)
     * @param runtime        The runtime var inherited from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * @param delayInSeconds The delay for this gate
     * @return Returns whether or not to let this gate through (true or false)
     */
    @Experimental
    public boolean synchronousDelayGateOPEN(String delayName, double runtime, double delayInSeconds) {
        if (delayInfoBools.get(delayName) == null) {
            delayInfoBools.put(delayName, false);
        }

        if (!delayInfoBools.get(delayName)) {
            delayInfoTimes.put(delayName, runtime);
            delayInfoBools.put(delayName, true);
        } else return runtime - delayInfoTimes.get(delayName) >= delayInSeconds;

        return false;
    }

    /**
     * Closes a delay gate
     *
     * @param delayName The name of the gate to close
     */
    @Experimental
    public void synchronousDelayGateCLOSE(String delayName) {
        delayInfoBools.put(delayName, false);
    }

    /**
     * Useful delay method for teleop opmodes -- want to delay a piece of code without stopping the thread entirely? Use THIS!
     * NOTE: This method will OPEN the gate for one update before closing it again
     *
     * @param delayName      The delay name (can be anything you want)
     * @param runtime        The runtime var inherited from {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}
     * @param delayInSeconds The delay for this gate
     * @return Returns whether or not to let this gate through (true or false)
     */
    @Experimental
    public boolean synchronousDelayGateCOMPLETE(String delayName, double runtime, double delayInSeconds) {
        if (delayInfoBools.get(delayName) == null) {
            delayInfoBools.put(delayName, false);
        }

        if (!delayInfoBools.get(delayName)) {
            delayInfoTimes.put(delayName, runtime);
            delayInfoBools.put(delayName, true);
        } else if (runtime - delayInfoTimes.get(delayName) >= delayInSeconds) {
            delayInfoTimes.put(delayName, runtime);
            return true;
        } else {
            delayInfoBools.put(delayName, false);
        }

        return false;
    }

    /**
     * Used to grab our automated controls
     *
     * @param controlName The name of the control specified in your mapping
     * @return The boolean value associated with the control
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Experimental
    public boolean getControl(String controlName) {
        ControllerMapping.ControlInput input = controlMap.get(controlName);
        if (input.controller != ControllerMapping.Controller.BOTH) {
            OptimizedController controller = input.controller == ControllerMapping.Controller.CONTROLLER1 ? controller1 : controller2;
            switch (input.type) {
                case BOOL: return controller.getBool(input.key);
                case PRESS: return controller.getOnPress(input.key);
                case TOGGLE: return controller.getToggle(input.key);
                case RELEASE: return controller.getOnRelease(input.key);
            }
        } else {
            switch (input.type) {
                case BOOL: return controller1.getBool(input.key) || controller2.getBool(input.key);
                case PRESS: return controller1.getOnPress(input.key) || controller2.getOnPress(input.key);
                case TOGGLE: return controller1.getToggle(input.key) ^ controller2.getToggle(input.key);
                case RELEASE: return controller1.getOnRelease(input.key) || controller2.getOnRelease(input.key);
            }
        }
        return false;
    }

    /**
     * Used to grab our automated controls
     *
     * @param controlName The name of the control specified in your mapping
     * @return The double value associated with the control
     */
    @Experimental
    public double getControlFloat(String controlName) {
        ControllerMapping.ControlInput input = controlMap.get(controlName);

        if (input.controller != ControllerMapping.Controller.BOTH) {
            OptimizedController controller = input.controller == ControllerMapping.Controller.CONTROLLER1 ? controller1 : controller2;
                return controller.getFloat(input.key);
        } else {
            return Math.abs(controller1.getFloat(input.key)) > Math.abs(controller2.getFloat(input.key)) ? controller1.getFloat(input.key) : controller2.getFloat(input.key);
        }
    }

    /**
     * Returns the data returned from {@link OptimizedOpenCVPipeline}
     *
     * @return The data
     * @usage Only call this method after calling initialize on the loader
     */
    public String getVisionOutputToken() {
        return loader.pipeline.getVisionOutput();
    }

    /**
     * Add a log to the register--won't show up on the phone until the {@link #pushLog()} method is called
     *
     * @param title The title of the log
     * @param value The value of data you want to log
     */
    public void autonomousLog(String title, Object value) {
        telemetry.addData("WABOT: " + title, value);
        telemetry.update();
    }

    /**
     * Print a log to the phone during a teleop opmode. Does NOT require the use of {@link #pushLog()}
     *
     * @param title The title of the log
     * @param value The value of data you want to log
     */
    public void teleopLog(String title, Object value) {
        telemetry.addData("WABOT: " + title, value);
    }

    /**
     * Push the autonomous logs, overriding any old messages with new ones
     * DO NOT USE during teleop (unless you know what you are doing)
     */
    public void pushLog() {
        telemetry.update();
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     *
     * @param defaultSpeedFactor The factor to multiple the speed by normally
     * @usage Only call this method after initialization and instantiating the robot
     * This algorithm makes gamepad 1 have standard forward controls over the robot!
     */
    public void updateDrive(OptimizedController controller1, OptimizedController controller2, double defaultSpeedFactor) {
        updateDrive(controller1, controller2, true, true, defaultSpeedFactor, 5d, 2d, RobotDirection.FRONT, RobotDirection.FRONT, false);
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     *
     * @param precisionModeFactor The factor to multiple the speed by if ONE of the bumpers are held down
     * @param slowmodeFactor      The factor to multiple the speed by if BOTH of the bumpers are held down
     * @param defaultSpeedFactor  The factor to multiple the speed by normally
     * @usage Only call this method after initialization and instantiating the robot
     * This algorithm makes gamepad 1 have standard forward controls over the robot!
     */
    public void updateDrive(OptimizedController controller1, OptimizedController controller2, double defaultSpeedFactor, double precisionModeFactor, double slowmodeFactor) {
        updateDrive(controller1, controller2, true, true, defaultSpeedFactor, precisionModeFactor, slowmodeFactor, RobotDirection.FRONT, RobotDirection.FRONT, false);
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     *
     * @param useController1         Can controller1 drive?
     * @param useController2         Can controller2 drive?
     * @param controller1Dir         Which direction should controller1 drive?
     * @param controller2Dir         Which direction should controller2 drive?
     * @param controller2CanOverride Can controller2 override controller1 using the {@link RobotConfig#NUCLEAR_KEY}?
     * @param defaultSpeedFactor     The factor to multiple the speed by normally
     * @usage Only call this method after initialization and instantiating the robot
     */
    @Experimental
    public void updateDrive(OptimizedController controller1, OptimizedController controller2, boolean useController1, boolean useController2, double defaultSpeedFactor, RobotDirection controller1Dir, RobotDirection controller2Dir, boolean controller2CanOverride) {
        updateDrive(controller1, controller2, useController1, useController2, defaultSpeedFactor, 0.7, 5d, controller1Dir, controller2Dir, controller2CanOverride);
    }

    /**
     * Our main driving method: uses one of a couple of drive algorithms to calculate and assign motor powers
     *
     * @param useController1         Can controller1 drive?
     * @param useController2         Can controller2 drive?
     * @param controller1Dir         Which direction should controller1 drive?
     * @param controller2Dir         Which direction should controller2 drive?
     * @param controller2CanOverride Can controller2 override controller1 using the {@link RobotConfig#NUCLEAR_KEY}?
     * @param defaultSpeedFactor     The factor to multiple the speed by normally
     * @param precisionModeFactor    The factor to multiple the speed by if ONE of the bumpers are held down
     * @param slowmodeFactor         The factor to multiple the speed by if BOTH of the bumpers are held down
     * @usage Only call this method after initialization and instantiating the robot
     */
    @Experimental
    public void updateDrive(OptimizedController controller1, OptimizedController controller2, boolean useController1, boolean useController2, double defaultSpeedFactor, double precisionModeFactor, double slowmodeFactor, RobotDirection controller1Dir, RobotDirection controller2Dir, boolean controller2CanOverride) {

        // If the OpMode didn't specifically initialize motors with settings, call the default one
        if (!hasUpdatedDrive && !hasInitializedMotors)
            initializeDriveMotors();
        hasUpdatedDrive = true;


        // Set drive state
        status = RobotStatus.DRIVING;

        // This is tuned to counteract imperfect strafing
        double strafingCo = 1.5;

        // Temporary Variables to be reassigned
        OptimizedController controller = controller1;
        RobotDirection direction = controller1Dir;

        //
        double x, y, rx;

        if (((!controller1.atRest() || !useController1) || (controller2CanOverride && controller2.getBool(RobotConfig.NUCLEAR_KEY))) && useController2) {
            controller = controller2;
            direction = controller2Dir;
        }

        double getX = controller.getFloat(OptimizedController.Key.LEFT_STICK_X) * strafingCo;
        double getY = controller.getFloat(OptimizedController.Key.LEFT_STICK_Y);
        if(!controller.getBool(OptimizedController.Key.DPAD_DOWN) && !controller.getBool(OptimizedController.Key.DPAD_UP) && !controller.getBool(OptimizedController.Key.DPAD_RIGHT) && !controller.getBool(OptimizedController.Key.DPAD_LEFT)) {
            x = (Math.abs(getX) < 0.1 && Math.abs(getY) > 0.6) ? 0 : getX;
            y = (Math.abs(getY) < 0.1 && Math.abs(getX) > 0.6) ? 0 : getY;
        } else {
            x = (controller.getBool(OptimizedController.Key.DPAD_LEFT)) ? -1 : ((controller.getBool(OptimizedController.Key.DPAD_RIGHT)) ? 1 : 0);
            y = (controller.getBool(OptimizedController.Key.DPAD_DOWN)) ? -1 : ((controller.getBool(OptimizedController.Key.DPAD_UP)) ? 1 : 0);
        }
        rx = controller.getFloat(OptimizedController.Key.RIGHT_STICK_X);

        if (direction == RobotDirection.FRONT) {
            y *= -1;
        } else if (direction == RobotDirection.BACK) {
            x *= -1;
        } else {
            x = y - x + (y = x);
            if (direction == RobotDirection.RIGHT) {
                y *= -1;
                x *= -1;
            }
        }

        // Power variables
        double fl, fr, bl, br;

        fl = y + x + rx;
        bl = y - x + rx;
        fr = y - x - rx;
        br = y + x - rx;

        // Making sure our speeds are in capped at -1, 1
        if (Math.abs(fl) > 1 || Math.abs(bl) > 1 ||
                Math.abs(fr) > 1 || Math.abs(fl) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(fl), Math.abs(bl));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(br), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }
        if (controller.getBool(OptimizedController.Key.LEFT_BUMPER) && controller.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
            fl *= defaultSpeedFactor * (1 / slowmodeFactor);
            fr *= defaultSpeedFactor * (1 / slowmodeFactor);
            bl *= defaultSpeedFactor * (1 / slowmodeFactor);
            br *= defaultSpeedFactor * (1 / slowmodeFactor);
        } else if (controller.getBool(OptimizedController.Key.LEFT_BUMPER) || controller.getBool(OptimizedController.Key.RIGHT_BUMPER)) {
            fl *= defaultSpeedFactor * (1 / precisionModeFactor);
            fr *= defaultSpeedFactor * (1 / precisionModeFactor);
            bl *= defaultSpeedFactor * (1 / precisionModeFactor);
            br *= defaultSpeedFactor * (1 / precisionModeFactor);
        } else {
            fl *= defaultSpeedFactor;
            fr *= defaultSpeedFactor;
            bl *= defaultSpeedFactor;
            br *= defaultSpeedFactor;
        }

        motors[1].setPower(fr);
        motors[0].setPower(fl);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }

    /**
     * Grab an encoder
     *
     * @param name Name of encoder
     */
    public Encoder getEncoder(String name) {
        return new Encoder(internalMap.get(DcMotorEx.class, name));
    }

    /**
     * Below is a series of methods to change the settings of the motors. Ignore mostly, these are all used in another method shown here:
     *
     * @param withEncoder
     * @link #initializeDriveMotors()
     */
    protected void runWithEncoder(boolean withEncoder) {
        if (withEncoder) {
            for (int i = 0; i < motors.length; i++) {
                if (!disabledMotors[i]) {
                    motors[i].setMode(RunMode.STOP_AND_RESET_ENCODER);
                    motors[i].setMode(RunMode.RUN_USING_ENCODER);
                }
            }
        } else {
            for (int i = 0; i < motors.length; i++) {
                if (!disabledMotors[i]) {
                    motors[i].setMode(RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
    }

    private void setMotorBrakeType(ZeroPowerBehavior type) {
        for (int i = 0; i < motors.length; i++) {
            if (!disabledMotors[i]) {
                motors[i].setZeroPowerBehavior(type);
            }
        }
    }

    protected void motorDir(boolean forward) {
        if (forward) {
            for (int i = 0; i < motors.length; i++) {
                if (!disabledMotors[i]) {
                    motors[i].setDirection(RobotConfig.motorDirections[i]);
                }
            }
        } else {
            for (int i = 0; i < motors.length; i++) {
                if (!disabledMotors[i]) {
                    if (RobotConfig.motorDirections[i] == Direction.FORWARD)
                        motors[i].setDirection(Direction.REVERSE);
                    else
                        motors[i].setDirection(Direction.FORWARD);
                }
            }
        }
    }

    /**
     * This method is used for testing purposes (if you want to swap another motor into the same stop as a drive motor, disable it so no error occurs)
     *
     * @param FLDisabled Whether or not the FLMotor is disabled
     * @param FRDisabled Whether or not the FRMotor is disabled
     * @param BLDisabled Whether or not the BLMotor is disabled
     * @param BRDisabled Whether or not the BRMotor is disabled
     */
    public void disableDriveMotors(boolean FLDisabled, boolean FRDisabled, boolean BLDisabled, boolean BRDisabled) {
        disabledMotors = new boolean[]{FLDisabled, FRDisabled, BLDisabled, BRDisabled};
    }

    /**
     * Returns the status of the robot. Currently has not much use but could be helpful in the future
     *
     * @return The Current Status of the Robot
     */
    public RobotStatus getStatus() {
        return status;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name The name of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotor getMotor(String name) {
        return getMotor(name, RunMode.RUN_WITHOUT_ENCODER, Direction.FORWARD, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name The name of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name) {
        return getMotorEx(name, RunMode.RUN_WITHOUT_ENCODER, Direction.FORWARD, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name    The name of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode) {
        return getMotor(name, runmode, Direction.FORWARD, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name The name of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode) {
        return getMotorEx(name, runmode, Direction.FORWARD, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode, Direction direction) {
        return getMotor(name, runmode, direction, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode, Direction direction) {
        return getMotorEx(name, runmode, direction, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Turns Rev BlinkinLed Lights into the confetti light pattern
     */
    @Experimental
    @OldCode
    public void runLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        RevBlinkinLedDriver ledLights = internalMap.get(RevBlinkinLedDriver.class, "light"/*check defined name*/);
        ledLights.setPattern(pattern);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotor motor = internalMap.dcMotor.get(name);
        if (runmode == RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(0);
        }
        motor.setMode(runmode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, RunMode runmode, ZeroPowerBehavior brakeMode) {
        return getMotor(name, runmode, Direction.FORWARD, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, Direction direction, ZeroPowerBehavior brakeMode) {
        return getMotor(name, RunMode.RUN_WITHOUT_ENCODER, direction, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, ZeroPowerBehavior brakeMode) {
        return getMotor(name, RunMode.RUN_WITHOUT_ENCODER, Direction.FORWARD, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotor(String name, Direction direction) {
        return getMotor(name, RunMode.RUN_WITHOUT_ENCODER, direction, ZeroPowerBehavior.BRAKE);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        DcMotorEx motor = internalMap.get(DcMotorEx.class, name);
        if (runmode == RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(0);
        }
        motor.setMode(runmode);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(brakeMode);
        return motor;
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, RunMode runmode, ZeroPowerBehavior brakeMode) {
        return getMotorEx(name, runmode, Direction.FORWARD, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, Direction direction, ZeroPowerBehavior brakeMode) {
        return getMotorEx(name, RunMode.RUN_WITHOUT_ENCODER, direction, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, ZeroPowerBehavior brakeMode) {
        return getMotorEx(name, RunMode.RUN_WITHOUT_ENCODER, Direction.FORWARD, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param name      The name of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorEx(String name, Direction direction) {
        return getMotorEx(name, RunMode.RUN_WITHOUT_ENCODER, direction, ZeroPowerBehavior.BRAKE);
    }


    /**
     * Searches through our map of aliases and looks for which hardware map name it falls under
     *
     * @param alias The alias of the hardware component
     * @return The true hardware name
     */
    private String findMapNameByAlias(String alias) {
        for (String key : aliasMap.keySet()) {
            if (alias.equals(key))
                return key;
            for (String val : aliasMap.get(key)) {
                if (val.equals(alias))
                    return key;
            }
        }
        return null;
    }


    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias The alias of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias) {
        return getMotor(findMapNameByAlias(alias));
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias   The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode) {
        return getMotor(findMapNameByAlias(alias), runmode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode, Direction direction) {
        return getMotor(findMapNameByAlias(alias), runmode, direction);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        return getMotor(findMapNameByAlias(alias), runmode, direction, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, RunMode runmode, ZeroPowerBehavior brakeMode) {
        return getMotor(findMapNameByAlias(alias), runmode, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, Direction direction, ZeroPowerBehavior brakeMode) {
        return getMotor(findMapNameByAlias(alias), direction, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, ZeroPowerBehavior brakeMode) {
        return getMotor(findMapNameByAlias(alias), brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotor getMotorByAlias(String alias, Direction direction) {
        return getMotor(findMapNameByAlias(alias));
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias The alias of the motor in the hardwareMap
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias) {
        return getMotorEx(findMapNameByAlias(alias));
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias   The alias of the motor in the hardwareMap
     * @param runmode The runmode to use for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode) {
        return getMotorEx(findMapNameByAlias(alias), runmode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode, Direction direction) {
        return getMotorEx(findMapNameByAlias(alias), runmode, direction);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode, Direction direction, ZeroPowerBehavior brakeMode) {
        return getMotorEx(findMapNameByAlias(alias), runmode, direction, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param runmode   The runmode to use for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, RunMode runmode, ZeroPowerBehavior brakeMode) {
        return getMotorEx(findMapNameByAlias(alias), runmode, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, Direction direction, ZeroPowerBehavior brakeMode) {
        return getMotorEx(findMapNameByAlias(alias), direction, brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param brakeMode The brake mode for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, ZeroPowerBehavior brakeMode) {
        return getMotorEx(findMapNameByAlias(alias), brakeMode);
    }

    /**
     * Returns a motor to be used in an OpMode (please do not use this for drive motors, as everything is handled here)
     *
     * @param alias     The alias of the motor in the hardwareMap
     * @param direction The direction for this motor
     * @return The DCMotor
     */
    public DcMotorEx getMotorExByAlias(String alias, Direction direction) {
        return getMotorEx(findMapNameByAlias(alias), direction);
    }

    /**
     * Returns a servo to be used in an OpMode
     *
     * @param name The hardwareMap name of the servo
     * @return The servo
     */
    public Servo getServo(String name) {
        return internalMap.servo.get(name);
    }

    /**
     * Returns a servo to be used in an OpMode
     *
     * @param name            The hardwareMap name of the servo
     * @param initialPosition The initialize position for this servo to assume
     * @return The servo
     */
    public Servo getServo(String name, double initialPosition) {
        Servo servo = internalMap.servo.get(name);
        servo.setPosition(initialPosition);
        return servo;
    }

    /**
     * Returns an ODS sensor to be used in an OpMode
     *
     * @param name The hardwareMap name of the ODS sensor
     * @return The ODS Sensor
     */
    public OpticalDistanceSensor getDistanceSensor(String name) {
        return internalMap.opticalDistanceSensor.get(name);
    }

    /**
     * Returns an instance of the DriveFunctions Class, which provides a series of primitive functions for autonomous
     *
     * @return Obj of the DriveFunctions class
     */
    public OptimizedDriveFunctions getDriveFunctions() {
        return functions;
    }

    /**
     * Returns the drive mode currently being used by the robot. Not much of a use for it, but kept just in-case.
     * To see a list of available drive modes, go here {@link DriveMode}
     *
     * @return An ENUM representing the drive mode
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Call this method to change the current drive mode of the robot
     *
     * @param mode An ENUM of type {@link DriveMode}
     * @usage You really should only be calling this method during initialization unless you are doing something special.
     */
    public void setDriveMode(DriveMode mode) {
        driveMode = mode;
    }

    /**
     * This method is very unique: If called explicitly before calling the first instance of updateDrive(), then it can be customized to the programmer's liking.
     * If not, the parameter-less version will run with default settings for teleop
     */
    protected void initializeDriveMotors() {
        initializeDriveMotors(true, false, ZeroPowerBehavior.BRAKE);
    }

    public void initializeDriveMotors(boolean isForward, boolean runsEncoders, ZeroPowerBehavior brakeType) {

        hasInitializedMotors = true;

        if (!disabledMotors[0]) {
            motors[0] = internalMap.dcMotor.get("frontLeftMotor");
        }
        if (!disabledMotors[1]) {
            motors[1] = internalMap.dcMotor.get("frontRightMotor");
        }
        if (!disabledMotors[2]) {
            motors[2] = internalMap.dcMotor.get("backLeftMotor");
        }
        if (!disabledMotors[3]) {
            motors[3] = internalMap.dcMotor.get("backRightMotor");
        }

        runWithEncoder(runsEncoders);

        motorDir(isForward);

        setMotorBrakeType(brakeType);
    }

    public void initializeDriveMotors(boolean isForward, boolean runsEncoders) {
        initializeDriveMotors(isForward, runsEncoders, ZeroPowerBehavior.BRAKE);
    }

    public void initializeDriveMotors(boolean isForward, ZeroPowerBehavior brakeType) {
        initializeDriveMotors(isForward, false, ZeroPowerBehavior.BRAKE);
    }

    public void initializeDriveMotors(boolean runsEncoders) {
        initializeDriveMotors(true, runsEncoders, ZeroPowerBehavior.BRAKE);
    }
}