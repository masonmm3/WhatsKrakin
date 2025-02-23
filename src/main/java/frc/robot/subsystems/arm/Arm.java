package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  // soft limits
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(7);
  public static final Rotation2d minExtend = Rotation2d.fromRotations(0);
  public static final Rotation2d maxExtend = Rotation2d.fromRotations(7);

  // Tunable numbers
  private static final LoggedTunableNumber kPPivot = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber kDPivot = new LoggedTunableNumber("Arm/kD");
  private static final LoggedTunableNumber kSPivot = new LoggedTunableNumber("Arm/kS");
  private static final LoggedTunableNumber kGPivot = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSecPivot =
      new LoggedTunableNumber("Arm/MaxVelocityDegreesPerSec", 500);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2Pivot =
      new LoggedTunableNumber("Arm/MaxAccelerationDegPerSec2", 2000);
  private static final LoggedTunableNumber staticCharacterizationVelocityThreshPivot =
      new LoggedTunableNumber("Arm/StaticCharaterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber kPExtend = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber kDExtend = new LoggedTunableNumber("Arm/kD");
  private static final LoggedTunableNumber kSExtend = new LoggedTunableNumber("Arm/kS");
  private static final LoggedTunableNumber kGExtend = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSecExtend =
      new LoggedTunableNumber("Arm/MaxVelocityDegreesPerSec", 500);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2Extend =
      new LoggedTunableNumber("Arm/MaxAccelerationDegPerSec2", 2000);
  private static final LoggedTunableNumber staticCharacterizationVelocityThreshExtend =
      new LoggedTunableNumber("Arm/StaticCharaterizationVelocityThresh", 0.1);

  static {
    switch (Constants.getRobot()) {
      case SIMBOT -> {
        kPPivot.initDefault(4000);
        kDPivot.initDefault(1000);
        kSPivot.initDefault(1.2);
        kGPivot.initDefault(0.0);
        kPExtend.initDefault(4000);
        kDExtend.initDefault(1000);
        kSExtend.initDefault(1.2);
        kGExtend.initDefault(0.0);
      }

      default -> {
        kPPivot.initDefault(0);
        kDPivot.initDefault(0);
        kSPivot.initDefault(0);
        kGPivot.initDefault(0);
        kPExtend.initDefault(0);
        kDExtend.initDefault(0);
        kSExtend.initDefault(0);
        kGExtend.initDefault(0);
      }
    }
  }

  // hardware
  private final ArmIO ArmIO;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // overrides
  private BooleanSupplier coastOverridePivot = () -> false;
  private BooleanSupplier disabledOverridePivot = () -> false;
  private BooleanSupplier coastOverrideExtend = () -> false;
  private BooleanSupplier disabledOverrideExtend = () -> false;

  @AutoLogOutput(key = "Arm/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Getter private boolean isEStopped = false;

  @Getter
  @AutoLogOutput(key = "Arm/Profile/AtGoal")
  private boolean atGoal = false;

  @AutoLogOutput private boolean hasCoral = false;

  //  @Setter private double extendVolts = 0.0;
  //  @Setter private double pivotAngle = 0.0;

  // private Debouncer armDebouncer = new Debouncer(0.1);

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Arm pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert extendMotorDisconnectedAlert =
      new Alert("Arm extender motor disconnected!", Alert.AlertType.kWarning);

  private final Alert disconnected;

  protected final Timer stateTimer = new Timer();

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO armIO) {
    this.armIO = armIO;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSecPivot.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2Pivot.get())));

    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
      new Trigger(() -> DriverStation.getStickButtonPressed(2,1))
        .onTrue(Commands.runOnce(() -> hasCoral = !hasCoral));
    }
  }

  public void periodic() {
    armIO.updateInputs(armInputs);
    Logger.processInputs("Arm", armInputs);

    pivotMotorDisconnectedAlert.set(
      !armInputs.data.motorConnected() && Constants.getRobot() == RobotType.COMPBOT
    );

    //update tunable numbers
    if (kPPivot.hasChanged(hashCode()) || kDPivot.hasChanged(hashCode())) {
      armIO.setPIDPivot(kPPivot.get(), 0.0, kDPivot.get());
    }
    if (kPExtend.hasChanged(hashCode()) || kDExtend.hasChanged(hashCode())) {
      armIO.setPIDExtend(kPExtend.get(), 0.0, kDExtend.get());
    }
    if (maxVelocityDegPerSecPivot.hasChanged(hashCode())
        || maxAccelerationDegPerSec2Pivot.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSecPivot.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2Pivot.get())));
    }

    //set coast mode
    setBrakeModePivot(!coastOverridePivot.getAsBoolean());
    setBrakeModeExtend(!coastOverridePivot.getAsBoolean());

  }

  @AutoLogOutput
  public Command runArm(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
  }
}
