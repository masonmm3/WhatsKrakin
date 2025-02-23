package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

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
  
  default void runPosition(Rotation2d position, double feedforward){}

  default void setPID(double kP, double kI, double kD){}

  default void setBrakeMode(boolean enabled) {}
}
