package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
  @AutoLog
  static class ArmIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double current = 0.0;
    public double voltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double supplyVoltageVolts = 0.0;
  }
  


  default void runTorqueCurrent(double current) {}

  default void runVolts(double volts) {}

  default void setBrakeMode(boolean brake) {}

  default void stop() {}

  default void updateInputs(ArmIOInputs inputs) {}
}
