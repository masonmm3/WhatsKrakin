package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
 @AutoLog
    public static class ArmIOInputs {
    public double ArmSetPoint = 0.0;
    public double ArmPositionError = 0.0;
    public double ArmSecondaryPosition = 0.0;
    public double ExtendVelocity = 0.0;
    public double AnglePosition = 0.0;
    public double AngleVelocity = 0.0;
    public double ArmResetCount = 0.0;
    public boolean ArmisOK = false;
    public boolean ExtendisOK = false;

    } 
    
    public default void setMotors(double extend, double angle) {
    }
    
}
