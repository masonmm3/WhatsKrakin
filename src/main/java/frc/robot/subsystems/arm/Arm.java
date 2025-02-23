package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm {
 
  private Pose3d angle = new Pose3d();

  private final ArmIO io;
  private final ArmIOInputsAutoLogOutput inputs = new ArmIOInputsAutoLogOutput();
  public double ArmAngle;
  public double ExtendLength;
  public Timer oneLaunch = new Timer();
  public int coralStack = 0;
  public Timer coralRecorder = new Timer(); 

  private Pose2d oldpose = new Pose2d();
  private Timer time = new Timer();

  public void advancedArm(boolean home,boolean Ninety, boolean twoSeventy, boolean oneAte) {
    if (home) {
      ArmAngle = 0;
    } else if (Ninety) {
      ArmAngle = 90;
    } else if (twoSeventy) {
      ArmAngle = 270;
    } else if (oneAte) {
      ArmAngle = 180;
    }

    io.setMotors(ExtendLength, ArmAngle);
  }

