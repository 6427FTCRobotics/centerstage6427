package org.firstinspires.ftc.teamcode.teamcode;

import org.firstinspires.ftc.teamcode.internal.ControllerMapping;
import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;

import java.util.HashMap;

@Experimental
public class CenterStageControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, ControlInput> initializeMapping(HashMap<String, ControlInput> mapping) {

        mapping.put("intake", new ControlInput(OptimizedController.Key.RIGHT_STICK_Y, Controller.CONTROLLER2, Type.FLOAT));
        mapping.put("liftControl", new ControlInput(OptimizedController.Key.LEFT_STICK_Y, Controller.CONTROLLER2, Type.FLOAT));
        mapping.put("toggleIntake", new ControlInput(OptimizedController.Key.A, Controller.CONTROLLER2, Type.TOGGLE));

        return mapping;
    }
}