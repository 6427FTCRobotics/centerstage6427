package org.firstinspires.ftc.teamcode.internal;


import java.util.HashMap;

/**
 * Holds a mapping on the controller with all the buttons and floats
 * @author Owen and Alex
 */
public interface ControllerMapping {

    enum Controller {
        CONTROLLER1, CONTROLLER2, BOTH
    }
    enum Type {
        TOGGLE, PRESS, RELEASE, BOOL, FLOAT
    }

    /**
     *
     */
    class ControlInput {
        OptimizedController.Key key;
        Controller controller;
        Type type;

        /**
         *
         * @param key The button on the controller to use
         * @param controller The controller to use
         * @param type The type
         */
        public ControlInput(OptimizedController.Key key, Controller controller, Type type) {
            this.key = key;
            this.controller = controller;
            this.type = type;
        }
    }

    /**
     * Build custom controls by inputting the name and control used for each hardware component
     * @return The mapping of device names to Keys on controller
     */
    HashMap<String, ControlInput> initializeMapping(HashMap<String, ControlInput> mapping);
}
