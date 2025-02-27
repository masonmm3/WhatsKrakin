package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ArmIOTalonFX implements ArmIO {
  public static final double reductionPivot = 7.0;
  public static final double reductionExtend = 10.0; // update
  private static final Rotation2d offsetPivot = new Rotation2d();
  private static final Rotation2d offsetExtend = new Rotation2d();

  // hardware
   private final CANBus kCANBus = new CANBus("rio");
  private final TalonFX talonPivot;
  private final TalonFX talonExtend;
  private final CANcoder canCoder;

  // config
  private final TalonFXConfiguration ConfigPivot = new TalonFXConfiguration();
  private final TalonFXConfiguration ConfigExtend = new TalonFXConfiguration();
  private final CANcoder configCANcoder = new CANcoder(0);

  // status signals
  private final StatusSignal<Angle> internalPositionPivot;
  private final StatusSignal<AngularVelocity> internalVelocityPivot;
  private final StatusSignal<Voltage> appliedVoltsPivot;
  private final StatusSignal<Current> supplyCurrentAmpsPivot;
  private final StatusSignal<Current> torqueCurrentAmpsPivot;
  private final StatusSignal<Temperature> tempPivot;

  private final StatusSignal<Angle> internalPositionExtend;
  private final StatusSignal<AngularVelocity> internalVelocityExtend;
  private final StatusSignal<Voltage> appliedVoltsExtend;
  private final StatusSignal<Current> supplyCurrentAmpsExtend;
  private final StatusSignal<Current> torqueCurrentAmpsExtend;
  private final StatusSignal<Temperature> tempExtend;

  // control requests
  private final TorqueCurrentFOC torqueCurrentFOCPivot =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOCPivot =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequestPivot = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final TorqueCurrentFOC torqueCurrentFOCExtend =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOCExtend =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequestExtend = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  

  // connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public ArmIOTalonFX() {
    talonPivot = new TalonFX(0, "*");
    talonExtend = new TalonFX(1, "*");

    // configure motors
    ConfigPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ConfigPivot.Slot0 = new Slot0Configs().withKG(0.0).withKP(0.0).withKI(0.0).withKD(0.0);
    ConfigPivot.Feedback.RotorToSensorRatio = reductionPivot;
    ConfigPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ConfigPivot.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    ConfigPivot.Feedback.SensorToMechanismRatio = reductionPivot;
    ConfigPivot.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    ConfigPivot.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    ConfigPivot.CurrentLimits.StatorCurrentLimit = 40.0;
    ConfigPivot.CurrentLimits.StatorCurrentLimitEnable = true;
    ConfigPivot.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talonPivot.getConfigurator().apply(ConfigPivot, 0.25));

    ConfigExtend.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ConfigExtend.Slot0 = new Slot0Configs().withKG(0.0).withKP(0.0).withKI(0.0).withKD(0.0);
    ConfigExtend.Feedback.RotorToSensorRatio = reductionExtend;
    ConfigExtend.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    ConfigExtend.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    ConfigExtend.Feedback.SensorToMechanismRatio = reductionExtend;
    ConfigExtend.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    ConfigExtend.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    ConfigExtend.CurrentLimits.StatorCurrentLimit = 40.0;
    ConfigExtend.CurrentLimits.StatorCurrentLimitEnable = true;
    ConfigExtend.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talonExtend.getConfigurator().apply(ConfigExtend, 0.25));

    // get and set signals
    internalPositionPivot = talonPivot.getPosition();
    internalVelocityPivot = talonPivot.getVelocity();
    appliedVoltsPivot = talonPivot.getMotorVoltage();
    supplyCurrentAmpsPivot = talonPivot.getSupplyCurrent();
    torqueCurrentAmpsPivot = talonPivot.getTorqueCurrent();
    tempPivot = talonPivot.getDeviceTemp();

    internalPositionExtend = talonExtend.getPosition();
    internalVelocityExtend = talonExtend.getVelocity();
    appliedVoltsExtend = talonExtend.getMotorVoltage();
    supplyCurrentAmpsExtend = talonExtend.getSupplyCurrent();
    torqueCurrentAmpsExtend = talonExtend.getTorqueCurrent();
    tempExtend = talonExtend.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        internalPositionPivot,
        internalVelocityPivot,
        appliedVoltsPivot,
        supplyCurrentAmpsPivot,
        torqueCurrentAmpsPivot,
        tempPivot,
        internalPositionExtend,
        internalVelocityExtend,
        appliedVoltsExtend,
        supplyCurrentAmpsExtend,
        torqueCurrentAmpsExtend,
        tempExtend);

    // register signals for refresh
    PhoenixUtil.registerSignals(
        internalPositionPivot,
        internalVelocityPivot,
        appliedVoltsPivot,
        supplyCurrentAmpsPivot,
        torqueCurrentAmpsPivot,
        tempPivot,
        internalPositionExtend,
        internalVelocityExtend,
        appliedVoltsExtend,
        supplyCurrentAmpsExtend,
        torqueCurrentAmpsExtend,
        tempExtend);

        MotionMagicConfigs extendMM = ConfigExtend.MotionMagic;
        extendMM.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
           .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
           // Take approximately 0.1 seconds to reach max accel 
           .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
     
           MotionMagicConfigs rotateMM = ConfigPivot.MotionMagic;
           rotateMM.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
             .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
             // Take approximately 0.1 seconds to reach max accel 
             .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
     
                 /* User can change the configs if they want, or leave it empty for factory-default */
                 configCANcoder.getConfigurator().apply(toApply);
     
         /* Speed up signals to an appropriate rate */
         BaseStatusSignal.setUpdateFrequencyForAll(100, configCANcoder.getPosition(), configCANcoder.getVelocity());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.data =
        new ArmIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    internalPositionPivot,
                    internalVelocityPivot,
                    appliedVoltsPivot,
                    supplyCurrentAmpsPivot,
                    torqueCurrentAmpsPivot,
                    tempPivot,
                    internalPositionExtend,
                    internalVelocityExtend,
                    appliedVoltsExtend,
                    supplyCurrentAmpsExtend,
                    torqueCurrentAmpsExtend,
                    tempExtend)),
            Rotation2d.fromRotations(internalPositionPivot.getValueAsDouble()),
            internalVelocityPivot.getValue().in(RadiansPerSecond),
            appliedVoltsPivot.getValue().in(Volts),
            supplyCurrentAmpsPivot.getValue().in(Amps),
            torqueCurrentAmpsPivot.getValue().in(Amps),
            tempPivot.getValue().in(Celsius),
            Rotation2d.fromRotations(internalPositionExtend.getValueAsDouble()),
            internalVelocityExtend.getValue().in(RadiansPerSecond),
            appliedVoltsExtend.getValue().in(Volts),
            supplyCurrentAmpsExtend.getValue().in(Amps),
            torqueCurrentAmpsExtend.getValue().in(Amps),
            tempExtend.getValue().in(Celsius));
  }

  @Override
  public void runOpenLoopPivot(double output) {
    talonPivot.setControl(torqueCurrentFOCPivot.withOutput(output));
  }

  @Override
  public void runOpenLoopExtend(double output) {
    talonExtend.setControl(torqueCurrentFOCExtend.withOutput(output));
  }

  @Override
  public void runVoltsPivot(double volts) {
    talonPivot.setControl(voltageRequestPivot.withOutput(volts));
  }

  @Override
  public void runVoltsExtend(double volts) {
    talonExtend.setControl(voltageRequestExtend.withOutput(volts));
  }

  @Override
  public void stop() {
    talonPivot.stopMotor();
    talonExtend.stopMotor();
  }

  @Override
  public void runPositionPivot(Rotation2d position, double feedforward) {
    talonPivot.setControl(
        positionTorqueCurrentFOCPivot
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPIDPivot(double kP, double kI, double kD) {
    ConfigPivot.Slot0.kP = kP;
    ConfigPivot.Slot0.kI = kI;
    ConfigPivot.Slot0.kD = kD;
    tryUntilOk(5, () -> talonPivot.getConfigurator().apply(ConfigPivot));
  }

  @Override
  public void setPIDExtend(double kP, double kI, double kD) {
    ConfigExtend.Slot0.kP = kP;
    ConfigExtend.Slot0.kI = kI;
    ConfigExtend.Slot0.kD = kD;
    tryUntilOk(5, () -> talonExtend.getConfigurator().apply(ConfigExtend));
  }

  @Override
  public void setBrakeModePivot(boolean enabled) {
    new Thread(
            () -> {
              ConfigPivot.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talonPivot.getConfigurator().apply(ConfigPivot));
            })
        .start();
  }

  @Override
  public void setBrakeModeExtend(boolean enabled) {
    new Thread(
            () -> {
              ConfigPivot.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talonPivot.getConfigurator().apply(ConfigPivot));
            })
        .start();
  }

  
  
}
