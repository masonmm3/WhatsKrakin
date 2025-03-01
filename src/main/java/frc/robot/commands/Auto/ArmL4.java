// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmL4 extends Command {
  /** Creates a new ArmL4. */
  private Timer timer = new Timer();

  private boolean scored;
  private boolean finished;

  public ArmL4() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
    scored = false;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 0.5) {
      RobotContainer.superStructure.setArm(SuperStructureConstants.L4Angle);
      RobotContainer.superStructure.setExtension(SuperStructureConstants.L4Extend);
    } else if (RobotContainer.superStructure.atSetpoint() && !scored) {
      RobotContainer.superStructure.setArm(
          SuperStructureConstants.L4Angle - SuperStructureConstants.scoreAngleDrop);
      RobotContainer.superStructure.setExtension(
          SuperStructureConstants.L4Extend - SuperStructureConstants.scoreExtendDrop);
      scored = true;
    } else if (RobotContainer.superStructure.atSetpoint() && scored) {
      RobotContainer.superStructure.setArm(SuperStructureConstants.HomeAngle);
      RobotContainer.superStructure.setExtension(SuperStructureConstants.HomeExtend);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.superStructure.setArm(SuperStructureConstants.HomeAngle);
    RobotContainer.superStructure.setExtension(SuperStructureConstants.HomeExtend);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.superStructure.atSetpoint() && finished;
  }
}
