package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOTalonFX implements ArmIO {

private final TalonFX armPivot;
private final TalonFX armExtension;

private final StatusSignal<Angle> position;
private final StatusSignal<AngularVelocity> velocity;
private final StatusSignal<Voltage> appliedVoltage;
private final StatusSignal<Current> supplyCurrentAmps;
private final StatusSignal<Current> torqueCurrent;
private final StatusSignal<Voltage> supplyVoltage;

private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
private final NeutralOut neutralOut = new NeutralOut();

private final double reductionPivot;
private final double reductionExtension;

private final Debouncer connectedDebouncer = new Debouncer(0.5);

public ArmIOTalonFX(
    int armPivotId,int armExtensionId, String bus, int currentLimitAmps, boolean invertPivot, boolean invertExtesion, boolean brake, double reductionPivot, double reductionExtension) {
this.reductionPivot = reductionPivot;
this.reductionExtension = reductionExtension;
armPivot = new TalonFX(armPivotId, bus);
armExtension = new TalonFX(armExtensionId, bus);

// Pivot Config
TalonFXConfiguration configPivot = new TalonFXConfiguration();
configPivot.MotorOutput.Inverted = invertPivot ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

configPivot.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
configPivot.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
configPivot.CurrentLimits.SupplyCurrentLimitEnable = true;
armPivot.getConfigurator().apply(configPivot);

position = armPivot.getPosition();
velocity = armPivot.getVelocity();
appliedVoltage = armPivot.getMotorVoltage();
supplyCurrentAmps = armPivot.getSupplyCurrent();
torqueCurrent = armPivot.getTorqueCurrent();
supplyVoltage = armPivot.getSupplyVoltage();
    
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVoltage, supplyCurrentAmps, torqueCurrent,supplyVoltage));
    armPivot.optimizeBusUtilization(0,1.0);

// // Extension Config
// TalonFXConfiguration configExtension = new TalonFXConfiguration();
// configExtension.MotorOutput.Inverted = invertExtesion ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; 

// configExtension.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
// configExtension.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
// configExtension.CurrentLimits.SupplyCurrentLimitEnable = true;
// armExtension.getConfigurator().apply(configExtension);
     }

@Override
public void updateInputs(ArmIOInputs inputs) {
inputs.connected = connectedDebouncer.calculate(BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, supplyCurrentAmps, torqueCurrent, supplyVoltage).isOK());
inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reductionPivot;
inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reductionPivot;
inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
inputs.supplyVoltageVolts = supplyVoltage.getValueAsDouble();
}

@Override
public void runTorqueCurrent(double current) {
armPivot.setControl(torqueCurrentFOC.withOutput(current));
}

@Override
public void runVolts(double volts) {
armPivot.setControl(voltageOut.withOutput(volts));
}

@Override
public void stop() {
    armPivot.setControl(neutralOut);
}

@Override
public void setBrakeMode(boolean enabled) {
    armPivot.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
}

}



    







}
