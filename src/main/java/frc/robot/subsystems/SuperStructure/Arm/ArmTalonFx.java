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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class ArmTalonFx implements ArmIO {
    private TalonFX _angleMotorK;
    private CANcoder _armEncoder;

    private static StatusSignal<Angle> _absolutePosition;
    private static StatusSignal<AngularVelocity> _armVelocity;

    public ArmTalonFx() {
        _angleMotorK = new TalonFX(0);
        _armEncoder = new CANcoder(0);

        var _armConfig = new TalonFXConfiguration();
        //current limits and ramp rates
        _armConfig.CurrentLimits.StatorCurrentLimit = 45;
        _armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        _armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        _armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
        _armConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
        //Closed loop settings
        _armConfig.ClosedLoopGeneral.ContinuousWrap = false;
        _armConfig.Feedback.FeedbackRemoteSensorID = _armEncoder.getDeviceID();
        _armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        _armConfig.Feedback.RotorToSensorRatio = 80/12;
        _armConfig.Feedback.SensorToMechanismRatio = 1;
        _armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        //GPID and VAS
        _armConfig.Slot0.kP = 0;
        _armConfig.Slot0.kI = 0;
        _armConfig.Slot0.kD = 0;
        _armConfig.Slot0.kG = 0;
        _armConfig.Slot0.kV = 0;
        _armConfig.Slot0.kS = 0;
        _armConfig.Slot0.kA = 0;
        //software limits
        _armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        _armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        _armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        _armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        //voltage limits
        _armConfig.Voltage.PeakForwardVoltage = 12;
        _armConfig.Voltage.PeakReverseVoltage = -12;

        _angleMotorK.getConfigurator().apply(_armConfig);

        _absolutePosition = _armEncoder.getAbsolutePosition();
        _armVelocity = _armEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100, _absolutePosition, _armVelocity
        );
        ParentDevice.optimizeBusUtilizationForAll(_angleMotorK);
    }

    @Override
    public void setAngle(double angle) {
        _angleMotorK.setControl(new PositionVoltage(angle).withSlot(0));
    }
}
