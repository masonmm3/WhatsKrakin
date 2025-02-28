// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Extension.Extension;

/** Add your docs here. */
public class SuperStructure {
  private Arm arm;
  private Extension extension;

  public SuperStructure(Arm arm, Extension extension) {
    this.arm = arm;
    this.extension = extension;
  }

  /**
 * updates superstructue values periodically
 */
public void structPeriodic() {
    arm.armPeriodic();
    extension.extensionPeriodic();
  }
}
