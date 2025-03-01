// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ArmSim implements ArmIO {
  private PIDController pid;
  private SingleJointedArmSim arm;

  public ArmSim() {
    arm =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            50,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), 5),
            Units.inchesToMeters(8),
            Units.degreesToRadians(-90),
            Units.degreesToRadians(270),
            false,
            0);
    pid = new PIDController(1, 0, 0.01);
  }

  @Override
  public void setAngle(double angle) {
    double goTo;
    if ((angle * 360 < SuperStructureConstants.PrepAngle
            && Units.radiansToDegrees(arm.getAngleRads()) > SuperStructureConstants.PrepAngle + 2)
        || (angle * 360 > SuperStructureConstants.PrepAngle
            && Units.radiansToDegrees(arm.getAngleRads())
                < SuperStructureConstants.PrepAngle
                    - 2)) { // protect against rotating the short way
      goTo = SuperStructureConstants.PrepAngle;
    } else {
      goTo = angle * 360;
    }
    double volts = MathUtil.clamp(pid.calculate(getAngle().getDegrees(), goTo), -12, 12);
    arm.setInputVoltage(volts);
  }

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d(arm.getAngleRads());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    arm.update(0.02);
    inputs.angle = getAngle().getDegrees();
  }
}
