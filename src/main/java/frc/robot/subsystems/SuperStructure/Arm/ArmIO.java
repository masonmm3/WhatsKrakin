// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog // Auto logs inputs
  public static class ArmIOInputs {
    public boolean connected = false;

    public double positionAngle = 0.0;
    public double velocityRPM = 0.0;

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {} // Updates inputs

  /**
   * @param angle go to angle in rotations
   */
  public default void setAngle(double angle) {}

  public default void runVolts(double volts) {}

  /**
   * gives current angle of actual arm
   *
   * @return rotation2d
   */
  public default Rotation2d getAngle() {
    return new Rotation2d();
  }
}
