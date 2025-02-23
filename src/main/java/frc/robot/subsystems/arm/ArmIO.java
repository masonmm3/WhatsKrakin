package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public ArmIOData data =
        new ArmIOData(false, Rotation2d.kZero, 0, 0, 0, 0, 0, Rotation2d.kZero, 0, 0, 0, 0, 0);
  }

  record ArmIOData(
      boolean motorsConnected,
      Rotation2d internalPositionPivot,
      double velocityPivotRadPerSec,
      double appliedVoltsPivot,
      double supplyCurrentAmpsPivot,
      double torqueCurrentAmpsPivot,
      double tempPivotCelsius,
      Rotation2d internalPositionExtend,
      double velocityExtendRadPerSec,
      double appliedVoltsExtend,
      double supplyCurrentAmpsExtend,
      double torqueCurrentAmpsExtend,
      double tempExtendCelsius) {}

  default void updateInputs(ArmIOInputs inputs) {}

  default void runOpenLoopPivot(double output) {}

  default void runOpenLoopExtend(double output) {}

  default void runVoltsPivot(double volts) {}

  default void runVoltsExtend(double volts) {}

  default void stop() {}

  default void runPositionPivot(Rotation2d position, double feedforward) {}

  default void setPIDPivot(double kP, double kI, double kD) {}

  default void setPIDExtend(double kP, double kI, double kD) {}

  default void setBrakeModePivot(boolean enabled) {}

  default void setBrakeModeExtend(boolean enabled) {}
}
