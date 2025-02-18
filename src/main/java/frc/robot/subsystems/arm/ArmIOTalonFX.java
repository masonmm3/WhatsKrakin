package frc.robot.subsystems.arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOTalonFX implements ArmIO {

//All Krakens
public static TalonFX AngleMotor = new TalonFX(0);
public static TalonFX ExtendMotor = new TalonFX(0);
public double armResetCount;
public double armSetPoint = 0.0;

//LEDs
public static Spark LedBlinkin = new Spark(0);

public ArmHardware() {

var extendConfig = new TalonFXConfiguration();
//Config
extendConfig.CurrentLimits.StatorCurrentLimit = 45;
extendConfig.CurrentLimits.StatorCurrentLimitEnable = true;
extendConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
// edit when we get arm working
extendConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
extendConfig.Slot0.kP = 0;
extendConfig.Slot0.kD = 0;
extendConfig.Slot0.kA = 0;
extendConfig.Slot0.kV = 0;
extendConfig.Slot0.kS = 0;

ExtendMotor.getConfigurator().apply(extendConfig);

var angleConfig = new TalonFXConfiguration();
//Config
angleConfig.CurrentLimits.StatorCurrentLimit = 40;
angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
angleConfig.Slot0.kP = 0;
angleConfig.Slot0.kD = 0;
angleConfig.Slot0.kG = 0;
angleConfig.Slot0.kV = 0;
angleConfig.Slot0.kS = 0;
angleConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
angleConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
angleConfig.MotionMagic.MotionMagicAcceleration = 0;
angleConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // 160 rps/s acceleration
angleConfig.MotionMagic.MotionMagicJerk = 0; // 1600 rps/s^2 jerk
angleConfig.Feedback.SensorToMechanismRatio = 12; // ReCheck
angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
angleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
angleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
angleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
angleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

AngleMotor.getConfigurator().apply(new TalonFXConfiguration());
AngleMotor.getConfigurator().apply(angleConfig);

absolutePosition = ArmEncoder.getAbsolutePosition();

BaseStatusSignal.setUpdateFrequencyForAll(250, absolutePosition);
ArmEncoder.optimizeBusUtilization();

armResetCount = 0.0;
}

@Override
public void updateInputs(ArmIOInputs inputs) {
 BaseStatusSignal.refreshAll(absolutePosition); 

inputs.ExtendVelocity = ExtendMotor.getVelocity().getValueAsDouble() * 60; // IDK why it is 60
inputs.AngleVelocity = AngleMotor.getVelocity().getValueAsDouble() * 60; // IDK why it is 60
inputs.AnglePosition = AngleMotor.getPosition().getValueAsDouble() * 360; // IDK why it is 360

inputs.ArmResetCount = armResetCount;

inputs.ArmPositionError = AngleMotor.getClosedLoopError().getValueAsDouble();
inputs.ArmSecondaryPosition = AngleMotor.getPosition().getValueAsDouble();

inputs.ArmisOK = AngleMotor.isAlive();
inputs.ExtendisOK = ExtendMotor.isAlive();

inputs.ArmSetPoint = armSetPoint;
}

}
