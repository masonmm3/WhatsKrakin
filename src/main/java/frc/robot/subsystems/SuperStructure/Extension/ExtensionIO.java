// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Extension;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ExtensionIO {

  @AutoLog
  public static class ExtensionIOInputs {
    public boolean connected = false;

    public double positionInch = 0.0;
    public double velocityRPM = 0.0;

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(ExtensionIOInputs inputs) {}

  /**
   * @param inch extends to distance in inches from start
   */
  public default void extendToDistance(double inch) {}

  public default void runVolts(double volts) {}

  public default double getExtend() {
    return 0;
  }
}
