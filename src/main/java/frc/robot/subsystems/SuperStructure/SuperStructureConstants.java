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

  public class otherSequence {
    public static String PrepareClimb = "Prepared climb";
    public static String Climbing = "Climbing";
    public static String ClimbHome = "Climb Home";
  }

  // Constants
  public static int ArmId = 21;
  public static int ArmEncoderId = 22;
  public static int ExtensionId = 20;
  public static int ExtensionEncoderID = 23;
  public static int ClimbId = 24;

  public static boolean ArmInvert = false;
  public static boolean ExtensionInvert = true;
  public static boolean ClimbInvert = false; // Needs to change possibly

  public static double anglePeakVoltage = 4;
  public static double extensionPeakVoltage = 6;
  public static double climbPeakVoltage = 6;

  public static double HomeAngle = -73.5; // -73.5
  public static double HomeExtend = 0; // 21.5
  public static double HomeClimb = 0; // Maybe add (Might need to change)

  // 12 inches from reef for our set points

  // Setpoints PLEASE USE POSITIVE NUMBERS -90 = 270
  public static double L4Extend = -21.7;
  public static double L3Extend = 21.4;
  public static double L2Extend = 26.4;
  public static double L1Extend = HomeExtend;

  public static double L4Angle = 106;
  public static double L3Angle = 126;
  public static double L2Angle = 161.5;
  public static double L1Angle = HomeAngle;

  public static double PrepAngle = 90; // should be straight up. (NEEDS TO BE STRAIGHT UP)
  public static double PrepExtend = 0.2;
  public static double PrepClimb = 0;

  public static double CollectPrepAngle = -73.5;
  public static double CollectPrepExtend = 21.5;

  public static double CollectAngle = -72.5;
  public static double CollectExtend = 15.4;

  public static double scoreAngleDrop = -12;
  public static double scoreExtendDrop = -7;
  public static double doClimb = 0;

  // closed loop
  public static double AngleP = 25; // 25
  public static double AngleI = 0;
  public static double AngleD = 3; // 3
  public static double AngleS = 0;
  public static double AngleV = 0;
  public static double AngleA = 0;
  public static double AngleG = 1.28;
  public static double angleSoftLimitLow = -90;
  public static double angleSoftLimitHigh = 270;
  public static double angleGearRatio = 84 / 12;

  public static double ExtensionP = 8.5; // 10
  public static double ExtensionI = 5; // 5
  public static double ExtensionD = 0;
  public static double ExtensionS = 0;
  public static double ExtensionV = 0;
  public static double ExtensionA = 0;
  public static double ExtensionG = 0;
  public static double extensionSoftLimitLow = 9 / (2 * Math.PI * 2);
  public static double extensionSoftLimitHigh = 40 / (2 * Math.PI * 2);
  public static double extensionGearRatio = 80 / 12;

  public static double ClimbP = 0;
  public static double ClimbI = 0;
  public static double ClimbD = 0;
  public static double ClimbS = 0;
  public static double ClimbV = 0;
  public static double ClimbA = 0;
  public static double ClimbG = 0;
  public static double climbSoftLimitLow = 0; // Change
  public static double climbSoftLimitHigh = 0; // Change
  public static double climbGearRatio = 180 / 1; // Gear Ratio 180/1
}
