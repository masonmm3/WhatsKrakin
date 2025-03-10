// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Extension;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Extension {
  private ExtensionIO io;
  private ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
  private double inch;

  public Extension(ExtensionIO io) {
    this.io = io;
  }

  /** updates extension values periodically */
  public void extensionPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Extension", inputs);
  }

  public void extendToDistance(double inch) {
    io.extendToDistance(inch);
    this.inch = inch;
  }

  public double getExtinsion() {
    return io.getExtend();
  }

  public boolean atExtension() {
    if (inch < (io.getExtend() + 0.5) && inch > io.getExtend() - 0.5) {
      return true;
    } else {
      return false;
    }
  }
}
