package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommands {
    public static Command simpleRotate(Arm arm) {
        return arm
            .run(arm.runOpenLoopPivot(0.1));
    }
}
