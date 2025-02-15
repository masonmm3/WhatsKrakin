package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  static class ArmIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
