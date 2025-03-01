// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ArmTalonFx implements ArmIO {
  private TalonFX _angleMotorK;
  private CANcoder _armEncoder;

  private static StatusSignal<Angle> _absolutePosition;
  private static StatusSignal<AngularVelocity> _armVelocity;

  public ArmTalonFx() {
    _angleMotorK = new TalonFX(SuperStructureConstants.ArmId);
    _armEncoder = new CANcoder(SuperStructureConstants.ArmEncoderId);

    var _armConfig = new TalonFXConfiguration();
    // current limits and ramp rates
    _armConfig.CurrentLimits.StatorCurrentLimit = 45;
    _armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    _armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    _armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    _armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    _armConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
    // Closed loop settings
    _armConfig.ClosedLoopGeneral.ContinuousWrap = true;
    _armConfig.Feedback.FeedbackRemoteSensorID = _armEncoder.getDeviceID();
    _armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    _armConfig.Feedback.RotorToSensorRatio = SuperStructureConstants.angleGearRatio;
    _armConfig.Feedback.SensorToMechanismRatio = 1;
    _armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // GPID and VAS
    _armConfig.Slot0.kP = SuperStructureConstants.AngleP;
    _armConfig.Slot0.kI = SuperStructureConstants.AngleI;
    _armConfig.Slot0.kD = SuperStructureConstants.AngleD;
    _armConfig.Slot0.kG = SuperStructureConstants.AngleG;
    _armConfig.Slot0.kV = SuperStructureConstants.AngleV;
    _armConfig.Slot0.kS = SuperStructureConstants.AngleS;
    _armConfig.Slot0.kA = SuperStructureConstants.AngleA;
    // software limits
    _armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.angleSoftLimitHigh;
    _armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.angleSoftLimitLow;
    // voltage limits
    _armConfig.Voltage.PeakForwardVoltage = 12;
    _armConfig.Voltage.PeakReverseVoltage = -12;

    _angleMotorK.getConfigurator().apply(_armConfig);

    _absolutePosition = _armEncoder.getAbsolutePosition();
    _armVelocity = _armEncoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100, _absolutePosition, _armVelocity);
    ParentDevice.optimizeBusUtilizationForAll(_angleMotorK);
  }

  @Override
  public void setAngle(double angle) {
    double goTo;
    if ((angle * 360 < SuperStructureConstants.PrepAngle
            && _absolutePosition.getValueAsDouble() * 360 > SuperStructureConstants.PrepAngle + 2)
        || (angle * 360 > SuperStructureConstants.PrepAngle 
            && _absolutePosition.getValueAsDouble()
                < SuperStructureConstants.PrepAngle
                    - 2)) { // protect against rotating the short way
      goTo = SuperStructureConstants.PrepAngle;
    } else {
      goTo = angle;
    }
    _angleMotorK.setControl(new PositionVoltage(goTo).withSlot(0));
  }

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d(
        Units.rotationsToRadians(_armEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  // TODO add Input logging
}
