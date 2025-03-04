// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

/** Add your docs here. */
public final class SuperStructureConstants {

  public class sequence {
    public static String L1 = "L1";
    public static String L2 = "L2";
    public static String L3 = "L3";
    public static String L4 = "L4";
    public static String Prep = "Prep";
    public static String Score = "Score";
    public static String Retract = "Retract";
    public static String Home = "Home";
  }

  // Constants
  public static int ArmId = 21;
  public static int ArmEncoderId = 22;
  public static int ExtensionId = 20;
  public static int ExtensionEncoderID = 23;

  public static boolean ArmInvert = false;
  public static boolean ExtensionInvert = true;

  public static double anglePeakVoltage = 4;
  public static double extensionPeakVoltage = 1;

  // Setpoints PLEASE USE POSITIVE NUMBERS -90 = 270
  public static double L4Extend = -10;
  public static double L3Extend = 0;
  public static double L2Extend = 0;
  public static double L1Extend = 0;

  public static double L4Angle = 95;
  public static double L3Angle = 0;
  public static double L2Angle = 0;
  public static double L1Angle = 0;

  public static double PrepAngle = 90; // should be straight up. (NEEDS TO BE STRAIGHT UP)
  public static double PrepExtend = -7;

  public static double CollectPrepAngle = -70;
  public static double CollectPrepExtend = -10;

  public static double CollectAngle = -70;
  public static double CollectExtend = -6;

  public static double scoreAngleDrop = -12;
  public static double scoreExtendDrop = -7;

  public static double HomeAngle = -70;
  public static double HomeExtend = -7;

  // closed loop
  public static double AngleP = 40;
  public static double AngleI = 0;
  public static double AngleD = 0;
  public static double AngleS = 0.1;
  public static double AngleV = 0.3;
  public static double AngleA = 0;
  public static double AngleG = 0;
  public static double angleSoftLimitLow = -90;
  public static double angleSoftLimitHigh = 270;
  public static double angleGearRatio = 84 / 12;

  public static double ExtensionP = 20;
  public static double ExtensionI = 0;
  public static double ExtensionD = 0;
  public static double ExtensionS = 0.01;
  public static double ExtensionV = 0.1;
  public static double ExtensionA = 0;
  public static double ExtensionG = 0;
  public static double ExtensionSoftLimitLow = 9 / (2 * Math.PI * 2);
  public static double ExtensionSoftLimitHigh = 40 / (2 * Math.PI * 2);
  public static double ExtensionGearRatio = 80 / 12;
}
