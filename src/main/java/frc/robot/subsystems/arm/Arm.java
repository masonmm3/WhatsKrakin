package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonGetter;
import com.fasterxml.jackson.annotation.JsonSetter;

public class Arm extends SubsystemBase {

  private final String name;
  private final ArmIO io;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  public Arm(String name, ArmIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start(); 


    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(7);
    public static final Rotation2d minExtend = Rotation2d.fromRotations(0);
    public static final Rotation2d maxExtend = Rotation2d.fromRotations(7);

    

    //Tunable numbers
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG");
    private static final LoggedTunableNumber maxVelocityDegPerSec = 
    new LoggedTunableNumber("Arm/MaxVelocityDegreesPerSec", 500);
    private static final LoggedTunableNumber maxAccelerationDegPerSec2 = 
    new LoggedTunableNumber("Arm/MaxAccelerationDegPerSec2", 2000);
    private static final LoggedTunableNumber staticCharacterizationVelocityThresh = 
    new LoggedTunableNumber("Arm/StaticCharaterizationVelocityThresh", 0.1);




    static {
      switch (Constants.getRobot()) {
        case SIMBOT -> {
          kP.initDefault(4000);
          kD.initDefault(1000);
          kS.initDefault(1.2);
          kG.initDefault(0.0);
        }
          
        default -> {
          kP.initDefault(0);
          kD.initDefault(0);
          kS.initDefault(0);
          kG.initDefault(0);
        }
          
      }
    }

    private final ArmIO pivotIo;
    private final ArmIOInputsAutoLogged pivotInputs = new ArmIOInputsAutoLogged();
    private final ArmIO extendIO;
    private final ArmIOInputsAutoLogged extendInputs =  new ArmIOInputsAutoLogged();

    private BooleanSupplier coastOverride = () -> false;
    private BooleanSupplier disabledOverride = () -> false;
    
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private DoubleSupplier goal = () -> 0.0;
    private boolean stopProfile = false;
    @Getter private boolean shouldEStop = false;
    @Setter private boolean isEStopped = false;

    // @Getter 
    // @AutoLogOutput (key = "Dispenser/Profile/AtGoal");
    // private boolean AtGoal = false;

    //  @Setter private double extendVolts = 0.0;
    //  @Setter private double pivotAngle = 0.0;


    private Debouncer armDebouncer = new Debouncer(0);

    private final Alert pivotMotorDisconnectedAlert = 
    new Alert("Arm pivot motor disconnected!", Alert.AlertType.kWarning);
    private final Alert pivotEncoderDisconnectedAlert =
    new Alert("Arm encoder disconnected!", Alert.AlertType.kWarning);
    private final Alert extendMotorDisconnectedAlert =
    new Alert("Arm extend motor disconnected!", Alert.AlertType.kWarning);

    public Arm(ArmIO pivotIo, ArmIO extendIO) {
      this.pivotIo = pivotIo;
      this.extendIO = extendIO;

      profile =
      new TrapezoidProfile(
        Units.degreesToRadians(maxVelocityDegPerSec.get()),
        Units.degreesToRadians(maxAccelerationDegPerSec2.get()));
    }
  }


    

  }

  public void periodic() {
    pivotIo.updateInputs(pivotInputs);
    Logger.processInputs("Arm/Pivot", pivotInputs);
    extendIO.updateInputs(extendInputs);
    Logger.processInputs("Arm/Extend", extendInputs);
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);
  }

  @AutoLogOutput
  public Command runArm(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
  }
}
