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
  private boolean starting;
  private double angle;
  private double extend;

  public ArmL4() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.restart();
    scored = false;
    finished = false;
    starting = true;
    angle = SuperStructureConstants.HomeAngle;
    extend = SuperStructureConstants.HomeExtend;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.superStructure.atSetpoint() && !starting && !scored) {
      angle -= SuperStructureConstants.scoreAngleDrop;
      extend -= SuperStructureConstants.scoreExtendDrop;
      scored = true;
      timer.restart();
    } else if (RobotContainer.superStructure.atSetpoint() && scored && timer.get() > 0.5 && !starting) {
      angle = SuperStructureConstants.HomeAngle;
      extend = SuperStructureConstants.HomeExtend;
      finished = true;
    } else if (!scored) {
      angle = SuperStructureConstants.L4Angle;
      extend = SuperStructureConstants.L4Extend;
      starting = false;
    }

    RobotContainer.superStructure.setArm(angle);
    RobotContainer.superStructure.setExtension(extend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.superStructure.setArm(angle);
    RobotContainer.superStructure.setExtension(extend);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.superStructure.atSetpoint() && finished && scored;
  }
}
