import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmMMCAN {
  private final double HEIGHT = 1;
  private final double WIDTH = 1;
  // Root Y and Y are for wrists

  private Mechanism2d rotate =
      new Mechanism2d(WIDTH, HEIGHT); // Adds a mechanism for Rotation by using the width and height

  // Arm root (pivot point where it rotates)
  private final MechanismLigament2d arm =
      rotate
          .getRoot("pivotPoint", 0.25, 0.5)
          .append(
              new MechanismLigament2d(
                  "arm", 0.2, 0, 6, new Color8Bit(Color.kAliceBlue))); // Length is variable

  @SuppressWarnings("unused")
  private final MechanismLigament2d leftArrow =
      arm.append(
          new MechanismLigament2d("LeftArrow", 0.1, 150, 6, new Color8Bit(Color.kAliceBlue)));

  @SuppressWarnings("unused")
  private final MechanismLigament2d rightArrow =
      arm.append(
          new MechanismLigament2d("RightArrow", 0.1, -150, 6, new Color8Bit(Color.kAliceBlue)));

  public void update(StatusSignal<Angle> rotationPosition, StatusSignal<Angle> extensionPosition) {
    // Set rotation angle (clock-like motion)
    arm.setAngle(rotationPosition.getValue().in(Rotations) * 360);

    // Set arm extension (scale it properly)
    arm.setLength(0.2 + extensionPosition.getValue().in(Rotations) * 0.5); // Adjust scaling factor

    // Update visualization
    SmartDashboard.putData("mech2d", rotate);
  }
  
}
