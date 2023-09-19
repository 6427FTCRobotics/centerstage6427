package org.firstinspires.ftc.teamcode.teamcode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.internal.ControllerMapping;
import org.firstinspires.ftc.teamcode.internal.Experimental;
import org.firstinspires.ftc.teamcode.internal.OptimizedController;

import java.util.HashMap;

@Experimental
public class FreightFrenzyControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, ControlInput> initializeMapping(HashMap<String, ControlInput> mapping) {

        /*
            Controls For Controller 1:
                - RT: Intake Float
                - LT: Outtake Float
                - RB: Intake Bool
                - LB: Outtake Bool
             Controls For Controller 2:
                - RSy: Move Arm Up and Down while arm is manual control mode
                - A: Toggle Arm Between Intake and Outtake Positions (default: Intake)
                - RS: Toggle Arm Between High And Low Positions (default: Low)
                - LB: Toggle Arm Between Front And Back Positions (default: Back)
                - X: Toggle Arm Between Automatic and Manual control Modes
                - LSy: Move Odometry Up and Down
                - RB: Duck Spinner Bool
                - RT: Duck Spinner Float
         */

        mapping.put("ArmSmooth", new ControlInput(OptimizedController.Key.RIGHT_STICK_Y, Controller.CONTROLLER2, Type.FLOAT));

        mapping.put("ArmOuttake", new ControlInput(OptimizedController.Key.A, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("ArmHeight", new ControlInput(OptimizedController.Key.RIGHT_STICK_BUTTON, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("ArmSide", new ControlInput(OptimizedController.Key.LEFT_BUMPER, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("ArmControl", new ControlInput(OptimizedController.Key.Y, Controller.CONTROLLER2, Type.TOGGLE));

        mapping.put("Odometry", new ControlInput(OptimizedController.Key.LEFT_STICK_Y, Controller.CONTROLLER2, Type.FLOAT));
        mapping.put("DuckReverse", new ControlInput(OptimizedController.Key.B, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("Duck", new ControlInput(OptimizedController.Key.X, Controller.CONTROLLER2, Type.TOGGLE));

        mapping.put("Intake", new ControlInput(OptimizedController.Key.RIGHT_TRIGGER, Controller.CONTROLLER1, Type.FLOAT));
        mapping.put("Outtake", new ControlInput(OptimizedController.Key.LEFT_TRIGGER, Controller.CONTROLLER1, Type.FLOAT));
        mapping.put("IntakeButton", new ControlInput(OptimizedController.Key.RIGHT_BUMPER, Controller.CONTROLLER1, Type.BOOL));
        mapping.put("OuttakeButton", new ControlInput(OptimizedController.Key.LEFT_BUMPER, Controller.CONTROLLER1, Type.BOOL));

        return mapping;
    }
}