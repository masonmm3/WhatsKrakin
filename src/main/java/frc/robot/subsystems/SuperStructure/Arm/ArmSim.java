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
        arm = new SingleJointedArmSim(DCMotor.getKrakenX60(1), SuperStructureConstants.angleGearRatio, 0.004, Units.inchesToMeters(5), 0, 0, true, 0, 0.04);
        pid = new PIDController(1, 0, 0);
    }

    @Override
    public void setAngle(double angle) {
        double goTo;
        if ((angle < SuperStructureConstants.PrepAngle && Units.radiansToDegrees(arm.getAngleRads()) > SuperStructureConstants.PrepAngle - 2) || (angle > SuperStructureConstants.PrepAngle && Units.radiansToDegrees(arm.getAngleRads()) < SuperStructureConstants.PrepAngle + 2)) { //protect against rotating the short way
            goTo = SuperStructureConstants.PrepAngle;
        } else {
            goTo = angle;
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
