// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

/** Add your docs here. */
public final class SuperStructureConstants {

  // Constants
  public static int ArmId = 21;
  public static int ArmEncoderId = 22;
  public static int ExtensionId = 20;
  public static int ExtensionEncoderID = 23;

  // Setpoints PLEASE USE POSITIVE NUMBERS -90 = 270
  public static double L4Extend = 0;
  public static double L3Extend = 0;
  public static double L2Extend = 0;
  public static double L1Extend = 0;

  public static double L4Angle = 30;
  public static double L3Angle = 20;
  public static double L2Angle = 10;
  public static double L1Angle = 0;

  public static double PrepAngle = 90; // should be straight up. (NEEDS TO BE STRAIGHT UP)
  public static double PrepExtend = 0;

  public static double CollectPrepAngle = 0;
  public static double CollectPrepExtend = 0;

  public static double CollectAngle = 0;
  public static double CollectExtend = 0;

  public static double scoreAngleDrop = 0;
  public static double scoreExtendDrop = 0;

  public static double HomeAngle = 0;
  public static double HomeExtend = 0;

  // closed loop
  public static double AngleP = 30; // 5
  public static double AngleI = 0;
  public static double AngleD = 0;
  public static double AngleS = 0;
  public static double AngleV = 0.7; // 0.1
  public static double AngleA = 0.02;
  public static double AngleG = 0.2; // 0.2
  public static double angleSoftLimitLow = 0;
  public static double angleSoftLimitHigh = 0;
  public static double angleGearRatio = 84 / 12;

  public static double ExtensionP = 0;
  public static double ExtensionI = 0;
  public static double ExtensionD = 0;
  public static double ExtensionS = 0;
  public static double ExtensionV = 0;
  public static double ExtensionA = 0;
  public static double ExtensionG = 0;
  public static double ExtensionSoftLimitLow = 0;
  public static double ExtensionSoftLimitHigh = 0;
  public static double ExtensionGearRatio = 80 / 12;
}
