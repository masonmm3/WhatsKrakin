package frc.robot.subsystems.SuperStructure.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean connected = false;

    public double positionClimb = 0.0;
    public double velocityRPM = 0.0;

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setClimb(double angleClimb) {}

  public default void runVolts(double volts) {}

  public default Rotation2d getClimb() {
    return new Rotation2d();
  }
}
