package frc.robot.subsystems.SuperStructure.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Climb {
  // Gear Ratio 180/1

  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged(); /*updates IO inputs
 /* and creates a place to update them */
  private double angle;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public void climbPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setClimbPosition(Rotation2d angleClimb) {
    io.setClimb(angleClimb.getRotations());
    /*  above, is this to get from rotations to degrees? I know it records something and processes it because its IO. */
    this.angle = angleClimb.getDegrees();
  }

  public Rotation2d getClimb() {
    return io.getClimb();
  }

  public boolean atClimb() {
    if (angle < (io.getClimb().getDegrees() + 1) && angle > io.getClimb().getDegrees() - 1) {
      return true;
    } else {
      return false;
    }
  }
}
