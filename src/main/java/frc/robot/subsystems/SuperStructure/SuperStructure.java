// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Extension.Extension;

/** Add your docs here. */
public class SuperStructure {
  private Arm arm;
  private Extension extension;
  private boolean debounce;
  private double armAngle;
  private double extendDistance;
  private String lastPose;

  public SuperStructure(Arm arm, Extension extension) {
    this.arm = arm;
    this.extension = extension;

    debounce = false;
    armAngle = 0;
    extendDistance = 0;

  }

  /**
 * updates superstructue values periodically
 */
  public void structPeriodic() {
    arm.armPeriodic();
    extension.extensionPeriodic();
  }

  /**
   * @param opA
   * @param opB
   * @param opY
   * @param opX
   * @param drRt
   * @param drLt
   * 
   * Runs the arm in simple teleop control mode (very dumb do not use outside extreme circumstance)
   */
  public void armTeleop(boolean opA, boolean opB, boolean opY, boolean opX, boolean drRt, double drLt) {
    if (opA) {
      armAngle = 0;
      extendDistance = 0;
    } else if (opB) {
      armAngle = 0;
      extendDistance = 0;
    } else if (opX) {
      armAngle = 0;
      extendDistance = 0;
    } else if (opY) {
      armAngle = 0;
      extendDistance = 0;
    } else if (drLt > 0.5) {
      armAngle = 0;
      extendDistance = 0;
    } else if (drLt < 0.5) {
      armAngle = 0;
      extendDistance = 0;
    }

    if(drRt && drLt < 0.1) { //drop angle to score
      armAngle -= 0;
    }

    arm.setPosition(new Rotation2d(Units.degreesToRadians(armAngle)));

    extension.extendToDistance(extendDistance);

  }

  /**
   * @param opA
   * @param opB
   * @param opY
   * @param opX
   * @param drRt
   * @param drLt
   * 
   * runs the arm in advanced mode runing each in a sequence
   */
  public void advancedArmTeleop(boolean opA, boolean opB, boolean opY, boolean opX, boolean drRt, double drLt) {

    //protect against multiple cases
    if (lastPose == "D" && arm.atTarget()) {
      extendDistance = 0;
      lastPose = "R";
    } else if (lastPose == "R" && arm.atTarget() && extension.atExtension()) {
      extendDistance = 0;
      armAngle = 0;
      lastPose = "";
    }
    //Go to prep pose (should be less than 180 from stow but within 90 of final target)
    else if (opA) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "A"; //should orbably be enum or vars in a constants class
    } else if (opB) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "B";
    } else if (opX) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "X";
    } else if (opY) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "Y";
    } else if (drLt > 0.5) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "Lt5";
    } else if (drLt < 0.5) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "Lt";
    }
    //actuall scoring pose to move to after releasing
    else if (!opA && lastPose == "A" && arm.atTarget()) { 
      armAngle = 0;
      extendDistance = 0;
      lastPose = "S";
    } else if (!opB && lastPose == "B" && arm.atTarget()) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "S";
    } else if (!opX && lastPose == "X" && arm.atTarget()) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "S";
    } else if (!opY && lastPose == "Y" && arm.atTarget()) {
      armAngle = 0;
      extendDistance = 0;
      lastPose = "S";
    }

    arm.setPosition(new Rotation2d(Units.degreesToRadians(armAngle)));

    extension.extendToDistance(extendDistance);
  }
}
