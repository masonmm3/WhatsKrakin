// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ExtensionSim implements ExtensionIO {
  public ElevatorSim extension;
  public PIDController controller;
  public ElevatorFeedforward feedforward;

  public ExtensionSim() {
    double[] stdDevs = new double[2];
    
    extension =
        new ElevatorSim(
            DCMotor.getKrakenX60(1), 6.66, 5, Units.inchesToMeters(4), 0, Units.inchesToMeters(75), true, 0);

    controller = new PIDController(1, 0, 0);
    feedforward = new ElevatorFeedforward(0.001, 0.03, 0.5);
  }

  @Override
  public void extendToDistance(double inch) {
    double pid = controller.calculate(getExtend(), inch);
    double feed = feedforward.calculate(inch - getExtend(), (inch - getExtend())/4);
    double volts =
        MathUtil.clamp(
            pid + feed,
            -SuperStructureConstants.extensionPeakVoltage,
            SuperStructureConstants.extensionPeakVoltage);
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
    inputs.connected = true;
    inputs.positionInch = getExtend();
    inputs.velocityRPM =
        Units.metersToInches(
            extension.getVelocityMetersPerSecond() / (2 * Math.PI * Units.inchesToMeters(4)));

    inputs.appliedVoltage = extension.getInput(0);
    inputs.supplyCurrentAmps = extension.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = extension.getCurrentDrawAmps();
    inputs.temperatureCelsius = 120; // everythings on fire
  }
}
