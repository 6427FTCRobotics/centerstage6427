package org.firstinspires.ftc.teamcode.internal;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * A class that adds a bunch of extra functionality to a normal gamepad, such as toggle buttons and onPress events.
 * Access these custom controllers through an instance of the OptimizedRobot class!
 *
 * @author Owen Boseley - Class of 2021
 */
public class OptimizedController {

    /**
     * Internal gamepad
     */
    private Gamepad internalGamepad;


    /**
     * Hashmaps used in storing internal boolean states for our gamepad keys
     */
    private final HashMap<Key, Boolean> canToggleList = new HashMap<>();
    private final HashMap<Key, Boolean> toggledList = new HashMap<>();
    private final HashMap<Key, Boolean> beforeStateList = new HashMap<>();

    /**
     * All the keys that are currently disabled on this controller
     */
    private final ArrayList<Key> keysDisabled = new ArrayList<>();

    /**
     * Constructor
     *
     * @param gamepad If assigned a value, on that gamepad can use this controller -- null if you want this to be optional
     */
    public OptimizedController(Gamepad gamepad) {
        this.internalGamepad = gamepad;
        for (Key key : Key.values()) {
            canToggleList.put(key, true);
        }
        for (Key key : Key.values()) {
            toggledList.put(key, false);
        }
        for (Key key : Key.values()) {
            beforeStateList.put(key, false);
        }
    }

    /**
     * The keys on the controller
     */
    public enum Key {
        START, A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, LEFT_TRIGGER, RIGHT_TRIGGER, LEFT_STICK_X, LEFT_STICK_Y, LEFT_STICK_BUTTON, RIGHT_STICK_X, RIGHT_STICK_Y, RIGHT_STICK_BUTTON, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, BACK
    }

    /**
     * Setter Method For {@link #toggledList }
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void setToggle(Key key, boolean value) {
        toggledList.replace(key, value);
    }

    /**
     * Are all analog sticks and triggers in their rest position?
     *
     * @return true if all analog sticks and triggers are at rest; otherwise false
     */
    public boolean atRest() {
        return internalGamepad.atRest();
    }

    public void vibrate() {
        internalGamepad.rumble(1000);
    }

    public void stopVibrate() {
        internalGamepad.stopRumble();
    }

    /**
     * Disable a key on the controller
     *
     * @param key        The key
     * @param isDisabled Whether or not you want this key to be disabled
     */
    public void disableKey(Key key, boolean isDisabled) {
        if (isDisabled)
            keysDisabled.add(key);
        else
            keysDisabled.remove(key);
    }

    /**
     * Returns whether the inputted key is disabled
     *
     * @param key The key to use
     * @return The state of disabled
     */
    private boolean isDisabled(Key key) {
        return keysDisabled.contains(key);
    }

    /**
     * Changes the gamepad associated with this controller
     */
    protected void setInternalGamepad(Gamepad gamepad) {
        this.internalGamepad = gamepad;
    }

    /**
     * Returns whether this key is traditionally a key that returns a number value associated with it
     *
     * @param key The key to use
     */
    public static boolean isFloatingTypeKey(Key key) {
        return key == Key.RIGHT_STICK_X || key == Key.RIGHT_STICK_Y || key == Key.LEFT_STICK_X || key == Key.LEFT_STICK_Y || key == Key.LEFT_TRIGGER || key == Key.RIGHT_TRIGGER;
    }

    /**
     * Toggles stored boolean value, for this key, when specified key is pressed
     *
     * @param key The key to use
     * @return The state of the toggle
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getToggle(Key key) {
        if (internalGamepad == null)
            return false;
        if (isDisabled(key))
            return false;
        if (getOnPress(key) && canToggleList.get(key)) {
            toggledList.replace(key, !toggledList.get(key));
        }
        return toggledList.get(key);
    }

    /**
     * Gets the boolean value of a key
     *
     * @param key The key to use
     * @return A boolean on whether or not this key is being pressed down
     */
    public boolean getBool(Key key) {
        if (internalGamepad == null)
            return false;
        if (isDisabled(key))
            return false;

        switch (key) {
            case A:
                return internalGamepad.a || internalGamepad.cross;
            case B:
                return internalGamepad.b || internalGamepad.circle;
            case X:
                return internalGamepad.x || internalGamepad.square;
            case Y:
                return internalGamepad.y || internalGamepad.triangle;
            case LEFT_BUMPER:
                return internalGamepad.left_bumper;
            case RIGHT_BUMPER:
                return internalGamepad.right_bumper;
            case DPAD_UP:
                return internalGamepad.dpad_up;
            case DPAD_DOWN:
                return internalGamepad.dpad_down;
            case DPAD_LEFT:
                return internalGamepad.dpad_left;
            case DPAD_RIGHT:
                return internalGamepad.dpad_right;
            case START:
                return internalGamepad.start || internalGamepad.options;
            case BACK:
                return internalGamepad.back || internalGamepad.share;
            case LEFT_STICK_X:
                return !NumberFunctions.isZero(internalGamepad.left_stick_x);
            case LEFT_STICK_Y:
                return !NumberFunctions.isZero(internalGamepad.left_stick_y);
            case LEFT_STICK_BUTTON:
                return internalGamepad.left_stick_button;
            case RIGHT_STICK_X:
                return !NumberFunctions.isZero(internalGamepad.right_stick_x);
            case RIGHT_STICK_Y:
                return !NumberFunctions.isZero(internalGamepad.right_stick_y);
            case RIGHT_STICK_BUTTON:
                return internalGamepad.right_stick_button;
            case LEFT_TRIGGER:
                return !NumberFunctions.isZero(internalGamepad.left_trigger);
            case RIGHT_TRIGGER:
                return !NumberFunctions.isZero(internalGamepad.right_trigger);
            default:
                return false;
        }
    }

    /**
     * Returns true for one frame when specified key is pressed
     *
     * @param key The key is use
     * @return The state
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getOnPress(Key key) {
        if (internalGamepad == null)
            return false;
        if (isDisabled(key))
            return false;
        if (!beforeStateList.get(key) && getBool(key)) {
            beforeStateList.replace(key, true);
            return true;
        } else {
            beforeStateList.replace(key, getBool(key));
            return false;
        }
    }

    /**
     * Returns true for one frame when specified key is released
     *
     * @param key The key is use
     * @return The state
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getOnRelease(Key key) {
        if (internalGamepad == null)
            return false;
        if (isDisabled(key))
            return false;

        if (beforeStateList.get(key) && !getBool(key)) {
            beforeStateList.replace(key, false);
            return true;
        } else {
            beforeStateList.replace(key, getBool(key));
            return false;
        }
    }

    /**
     * Gets the float value of keys on the controller (such as triggers or sticks)
     *
     * @param key The key to use
     * @return The float value returned from this key
     */
    public float getFloat(Key key) {
        if (internalGamepad == null)
            return 0;
        if (isDisabled(key))
            return 0;

        switch (key) {
            case LEFT_STICK_X:
                return internalGamepad.left_stick_x;
            case LEFT_STICK_Y:
                return internalGamepad.left_stick_y;
            case RIGHT_STICK_X:
                return internalGamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return internalGamepad.right_stick_y;
            case LEFT_TRIGGER:
                return internalGamepad.left_trigger;
            case RIGHT_TRIGGER:
                return internalGamepad.right_trigger;
            case A:
                return internalGamepad.a ? 1 : 0;
            case B:
                return internalGamepad.b ? 1 : 0;
            case X:
                return internalGamepad.x ? 1 : 0;
            case Y:
                return internalGamepad.y ? 1 : 0;
            case LEFT_BUMPER:
                return internalGamepad.left_bumper ? 1 : 0;
            case RIGHT_BUMPER:
                return internalGamepad.right_bumper ? 1 : 0;
            case DPAD_UP:
                return internalGamepad.dpad_up ? 1 : 0;
            case DPAD_DOWN:
                return internalGamepad.dpad_down ? 1 : 0;
            case DPAD_LEFT:
                return internalGamepad.dpad_left ? 1 : 0;
            case DPAD_RIGHT:
                return internalGamepad.dpad_right ? 1 : 0;
            case START:
                return internalGamepad.start ? 1 : 0;
            case BACK:
                return internalGamepad.back ? 1 : 0;
            default:
                return 0;
        }
    }
}