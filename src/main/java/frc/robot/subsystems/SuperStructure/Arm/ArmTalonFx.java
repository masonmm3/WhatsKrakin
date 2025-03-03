// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
 import com.ctre.phoenix6.hardware.ParentDevice;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ArmTalonFx implements ArmIO {

  private final TalonFX _angleMotorK;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final PositionVoltage positonOut = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  public ArmTalonFx() {
    _angleMotorK = new TalonFX(SuperStructureConstants.ArmId);
    position = _angleMotorK.getPosition();
    velocity = _angleMotorK.getVelocity();
    voltage = _angleMotorK.getMotorVoltage();
    supplyCurrentAmps = _angleMotorK.getSupplyCurrent();
    torqueCurrentAmps = _angleMotorK.getTorqueCurrent();
    tempCelsius = _angleMotorK.getDeviceTemp();


    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(SuperStructureConstants.ArmInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);
    cfg.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    cfg.Slot0.kP = SuperStructureConstants.AngleP;
    cfg.Slot0.kI = SuperStructureConstants.AngleI;
    cfg.Slot0.kD = SuperStructureConstants.AngleD;
    cfg.Slot0.kG = SuperStructureConstants.AngleG;
    cfg.Slot0.kV = SuperStructureConstants.AngleV;
    cfg.Slot0.kS = SuperStructureConstants.AngleS;
    cfg.Slot0.kA = SuperStructureConstants.AngleA;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SuperStructureConstants.angleSoftLimitHigh;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SuperStructureConstants.angleSoftLimitLow;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.Feedback.SensorToMechanismRatio = SuperStructureConstants.angleGearRatio;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // voltage limits
    cfg.Voltage.PeakForwardVoltage = SuperStructureConstants.anglePeakVoltage;
    cfg.Voltage.PeakReverseVoltage = -SuperStructureConstants.anglePeakVoltage;
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius);
    _angleMotorK.optimizeBusUtilization(0.0, 1.0);

    _angleMotorK.getConfigurator().apply(cfg);

  }

  @Override
  public void setAngle(double angle) {
    double goTo;
    if ((angle * 360 < SuperStructureConstants.PrepAngle

            && _angleMotorK.getPosition().getValueAsDouble() * 360
                > SuperStructureConstants.PrepAngle + 2)
        || (angle * 360 > SuperStructureConstants.PrepAngle
            && _angleMotorK.getPosition().getValueAsDouble()
                < SuperStructureConstants.PrepAngle
                    - 2)) { // protect against rotating the short way
      goTo = SuperStructureConstants.PrepAngle;
    } else {
      goTo = angle;
    }
    _angleMotorK.setControl(positonOut.withPosition(goTo).withSlot(0));
  }

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d(Units.rotationsToRadians(_angleMotorK.getPosition().getValueAsDouble()));
  }

  @Override
  public void runVolts(double volts) {
    _angleMotorK.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius)
            .isOK();

    inputs.positionAngle = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.getValueAsDouble());

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();

  }
  // TODO add Input logging
}
