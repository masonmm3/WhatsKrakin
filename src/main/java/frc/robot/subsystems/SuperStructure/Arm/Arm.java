// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Arm {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs;

  public Arm(ArmIO io) {
    this.io = io;
  }

  /**
  * updates arm values periodically
  */
  public void armPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Pivot", inputs);
  }

  public void setPosition(Rotation2d Angle) {
    io.setAngle(Angle);
  }
}
