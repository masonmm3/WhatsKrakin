// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ExtensionSim implements ExtensionIO {
  public ElevatorSim extension;
  public PIDController controller;

  public ExtensionSim() {
    double[] stdDevs = new double[2];
    stdDevs[0] = 0.004;
    stdDevs[1] = 0.004;
    extension =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            20,
            5,
            Units.inchesToMeters(4),
            0,
            4,
            false,
            0,
            stdDevs);

    controller = new PIDController(5, 0, 0);
  }

  @Override
  public void extendToDistance(double inch) {
    double volts = MathUtil.clamp(controller.calculate(getExtend(), inch), -12, 12);
    extension.setInputVoltage(volts);
  }

  @Override
  public double getExtend() {
    return Units.metersToInches(
        extension
            .getPositionMeters()); // should be the distance because CTRE Mechanisms good like that
  }

  // TODO zero using absolute encoder
  // TODO add input loggging
  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    extension.update(0.02);
    inputs.extend = getExtend();
  }
}
