package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
public Timer oneLaunch = new Timer;
public int coralStack = 0;
public Timer coralRecorder = new Timer 

private Pose2d oldpose = new Pose2d();
private Timer time = new Timer();

ch (ShooterMode.get()){
se Event:

if (Zero){
ArmAngle = 0;
ExtendLength = .1;

} else if (Ninety){
ArmAngle = 90;
ExtendLength = .5;
} else if (one-eighty){
ArmAngle = 180;
ExtendLength = 0;
} else if (two-seventy){
ArmAngle = 270;
ExtendLength = .5;
}
break;

default:

if (Ninety){
ArmAngle = 90;
ExtendLength = .5;
} else if (one-eighty){
ArmAngle = 180;
ExtendLength = 0;
} else if (two-seventy){
ArmAngle = 270;
ExtendLength = .5;
} else if (Zero){
ArmAngle = 0;
ExtendLength = .1;}



}