// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Extension;

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
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ExtensionTalonFx implements ExtensionIO {
        private TalonFX _extendMotorK;
    private CANcoder _extendEncoder;

    private static StatusSignal<Angle> _absolutePosition;
    private static StatusSignal<AngularVelocity> _extendVelocity;

    public ExtensionTalonFx() {
        _extendMotorK = new TalonFX(SuperStructureConstants.ExtensionId);
        _extendEncoder = new CANcoder(SuperStructureConstants.ExtensionEncoderID);

        var _extendConfig = new TalonFXConfiguration();
        //current limits and ramp rates
        _extendConfig.CurrentLimits.StatorCurrentLimit = 45;
        _extendConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _extendConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        _extendConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        _extendConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
        _extendConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
        //Closed loop settings
        _extendConfig.ClosedLoopGeneral.ContinuousWrap = false;
        _extendConfig.Feedback.FeedbackRemoteSensorID = _extendEncoder.getDeviceID();
        _extendConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        double extendInchToRotations = (2 * Math.PI * 4) * 80/12; // Find circumference and multiply by ratio. Idea (2 rotation = 10 inch, 6.6 Motor rotations = 1 rotation so 13.2 rotations = 10 inch)

        _extendConfig.Feedback.RotorToSensorRatio = extendInchToRotations;
        _extendConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        //GPID and VAS
        _extendConfig.Slot0.kP = SuperStructureConstants.ExtensionP;
        _extendConfig.Slot0.kI = SuperStructureConstants.ExtensionI;
        _extendConfig.Slot0.kD = SuperStructureConstants.ExtensionD;
        _extendConfig.Slot0.kG = SuperStructureConstants.ExtensionG;
        _extendConfig.Slot0.kV = SuperStructureConstants.ExtensionV;
        _extendConfig.Slot0.kS = SuperStructureConstants.ExtensionS;
        _extendConfig.Slot0.kA = SuperStructureConstants.ExtensionA;
        //software limits
        _extendConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        _extendConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SuperStructureConstants.ExtensionSoftLimitHigh;
        _extendConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        _extendConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SuperStructureConstants.ExtensionSoftLimitLow;
        //voltage limits
        _extendConfig.Voltage.PeakForwardVoltage = 12;
        _extendConfig.Voltage.PeakReverseVoltage = -12;

        _extendMotorK.getConfigurator().apply(_extendConfig);

        _absolutePosition = _extendEncoder.getAbsolutePosition();
        _extendVelocity = _extendEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100, _absolutePosition, _extendVelocity
        );
        ParentDevice.optimizeBusUtilizationForAll(_extendMotorK);
    }

    @Override
    public void extendToDistance(double inch) {
        _extendMotorK.setControl(new PositionVoltage(inch).withSlot(0));
    }

    @Override
    public double getExtend() {
        return _extendMotorK.getPosition().getValueAsDouble(); //should be the distance because CTRE Mechanisms good like that
    }

    //TODO zero using absolute encoder
    //TODO add input loggging
}
