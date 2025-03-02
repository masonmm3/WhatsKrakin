// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Arm {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double Angle;

  public Arm(ArmIO io) {
    this.io = io;
  }

  /** updates arm values periodically */
  public void armPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Pivot", inputs);
  }

  public void setPosition(Rotation2d Angle) {
    io.setAngle(Angle.getRotations());
    this.Angle = Angle.getDegrees();
  }

  public Rotation2d getAngle() {
    return io.getAngle();
  }

  public boolean atTarget() {
    if (Angle < (io.getAngle().getDegrees() + 1) && Angle > io.getAngle().getDegrees() - 1) {
      return true;
    } else {
      return false;
    }
  }
}
