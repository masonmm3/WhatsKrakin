package frc.robot.subsystems.arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

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

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;

  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final double reductionPivot;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ArmIOTalonFX(
      int idPivot,
      String bus,
      int currentLimitAmps,
      boolean invertPivot,
      boolean brakePivot,
      double reductionPivot) {
    this.reductionPivot = reductionPivot;
    armPivot = new TalonFX(idPivot, bus);

    TalonFXConfiguration configPivot = new TalonFXConfiguration();
    configPivot.MotorOutput.Inverted =
        invertPivot ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    configPivot.MotorOutput.NeutralMode =
        brakePivot ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    configPivot.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    configPivot.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> armPivot.getConfigurator().apply(configPivot));

    position = armPivot.getPosition();
    velocity = armPivot.getVelocity();
    appliedVoltage = armPivot.getMotorVoltage();
    supplyCurrent = armPivot.getSupplyCurrent();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent));
    tryUntilOk(5, () -> armPivot.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, supplyCurrent).isOK());
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reductionPivot;
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / reductionPivot;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
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
    new Thread(
            () ->
                tryUntilOk(
                    5,
                    () ->
                        armPivot.setNeutralMode(
                            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)))
        .start();
  }
}
