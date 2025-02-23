package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */

public final class Constants {
  // public static final Mode simMode = Mode.SIM;
  // public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double loopPeriodSecs = 0.02;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final boolean tuningMode = false;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
}
