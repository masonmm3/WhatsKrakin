package frc.robot.subsystems.arm;

public class armIO {
@AutoLog 
static class armIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double current = 0.0;
    public double voltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double supplyVoltageVolts = 0.0;
}
default void updateInputs(armIOInputs inputs) {}

default void runTorqueCurrent(double current) {}

default void runVolts(double volts) {}

default void stop() {}

default void setBrakeMode(boolean brake) {}





}
