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
  private double armAngle;
  private double extendDistance;
  private String lastPose;

  public SuperStructure(Arm arm, Extension extension) {
    this.arm = arm;
    this.extension = extension;

    lastPose = "";
    armAngle = 0;
    extendDistance = 0;
  }

  /** updates superstructue values periodically */
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
   *     <p>Runs the arm in simple teleop control mode (very dumb do not use outside extreme
   *     circumstance)
   */
  public void armTeleop(
      boolean opA, boolean opB, boolean opY, boolean opX, boolean drRt, double drLt) {
    if (opA) {
      armAngle = SuperStructureConstants.L1Angle;
      extendDistance = SuperStructureConstants.L1Extend;
    } else if (opX) {
      armAngle = SuperStructureConstants.L2Angle;
      extendDistance = SuperStructureConstants.L2Extend;
    } else if (opB) {
      armAngle = SuperStructureConstants.L3Angle;
      extendDistance = SuperStructureConstants.L3Extend;
    } else if (opY) {
      armAngle = SuperStructureConstants.L4Angle;
      extendDistance = SuperStructureConstants.L4Extend;
    } else if (drLt > 0.1) {
      armAngle = SuperStructureConstants.CollectPrepAngle;
      extendDistance = SuperStructureConstants.CollectPrepExtend;
    } else if (drLt > 0.75) {
      armAngle = SuperStructureConstants.CollectAngle;
      extendDistance = SuperStructureConstants.CollectExtend;
    }

    if (drRt && drLt < 0.1) { // drop angle to score
      armAngle -= SuperStructureConstants.scoreAngleDrop;
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
   *     <p>runs the arm in advanced mode runing each in a sequence
   */
  public void advancedArmTeleop(
      boolean opA,
      boolean opB,
      boolean opY,
      boolean opX,
      boolean drRt,
      double drLt,
      boolean opLb,
      boolean opRb) {

    // protect against multiple cases
    if ((opLb && opRb)) {
      armAngle = SuperStructureConstants.HomeAngle;
      extendDistance = SuperStructureConstants.HomeExtend;
      lastPose = "";
    }
    else if (lastPose == "S" && arm.atTarget() && drRt) {
      extendDistance -= SuperStructureConstants.scoreExtendDrop;
      armAngle -= SuperStructureConstants.scoreAngleDrop;
      lastPose = "D";
    } else if (lastPose == "D" && arm.atTarget() && !drRt) {
      extendDistance = SuperStructureConstants.HomeExtend;
      lastPose = "R";
    } else if (lastPose == "R" && arm.atTarget() && extension.atExtension()) {
      extendDistance = SuperStructureConstants.HomeExtend;
      armAngle = SuperStructureConstants.HomeAngle;
      lastPose = "";
    } // Go to prep pose (should be less than 180 from stow but within 90 of final target)
    else if (opA) {
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      lastPose = "A"; // should orbably be enum or vars in a constants class
    } else if (opB) {
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      lastPose = "B";
    } else if (opX) {
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      lastPose = "X";
    } else if (opY) {
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      lastPose = "Y";
    } else if (drLt > 0.75) {
      armAngle = SuperStructureConstants.CollectAngle;
      extendDistance = SuperStructureConstants.CollectExtend;
      lastPose = "";
    } else if (drLt > 0.1) {
      armAngle = SuperStructureConstants.CollectPrepAngle;
      extendDistance = SuperStructureConstants.CollectPrepExtend;
      lastPose = "";
    } // actuall scoring pose to move to after releasing
    else if (!opA && lastPose == "A" && arm.atTarget()) {
      armAngle = SuperStructureConstants.L1Angle;
      extendDistance = SuperStructureConstants.L1Extend;
      lastPose = "S";
    } else if (!opB && lastPose == "B" && arm.atTarget()) {
      armAngle = SuperStructureConstants.L3Angle;
      extendDistance = SuperStructureConstants.L3Extend;
      lastPose = "S";
    } else if (!opX && lastPose == "X" && arm.atTarget()) {
      armAngle = SuperStructureConstants.L2Angle;
      extendDistance = SuperStructureConstants.L2Extend;
      lastPose = "S";
    } else if (!opY && lastPose == "Y" && arm.atTarget()) {
      armAngle = SuperStructureConstants.L4Angle;
      extendDistance = SuperStructureConstants.L4Extend;
      lastPose = "S";
    } // Home Pose
    else if (lastPose == "" || (opLb && opRb)) {
      armAngle = SuperStructureConstants.HomeAngle;
      extendDistance = SuperStructureConstants.HomeExtend;
      lastPose = "";
    }

    arm.setPosition(new Rotation2d(Units.degreesToRadians(armAngle)));

    extension.extendToDistance(extendDistance);
  }

  public void setArm(double angle) {
    arm.setPosition(new Rotation2d(Units.degreesToRadians(angle)));
  }

  public void setExtension(double extend) {
    extension.extendToDistance(extend);
  }

  public boolean atSetpoint() {
    return arm.atTarget() && extension.atExtension();
  }
}
