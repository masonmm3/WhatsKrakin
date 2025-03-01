// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

/** Add your docs here. */
public final class SuperStructureConstants {

    public static int ArmId = 0;
    public static int ArmEncoderId = 0;
    public static int ExtensionId = 0;
    public static int ExtensionEncoderID = 0;


    //Setpoints
    public static double L4Extend = 0;
    public static double L3Extend = 0;
    public static double L2Extend = 0;
    public static double L1Extend = 0;

    public static double L4Angle = 0;
    public static double L3Angle = 0;
    public static double L2Angle = 0;
    public static double L1Angle = 0;

    public static double PrepAngle = 0; //should be straight up.
    public static double PrepExtend = 0;

    public static double CollectPrepAngle = 0;
    public static double CollectPrepExtend = 0;

    public static double CollectAngle = 0;
    public static double CollectExtend = 0;

    public static double scoreAngleDrop = 0;
    public static double scoreExtendDrop = 0;

    public static double HomeAngle = 0;
    public static double HomeExtend = 0;

    //closed loop
    public static double AngleP = 0;
    public static double AngleI = 0;
    public static double AngleD = 0;
    public static double AngleS = 0;
    public static double AngleV = 0;
    public static double AngleA = 0;
    public static double AngleG = 0;
    public static double angleSoftLimitLow = 0;
    public static double angleSoftLimitHigh = 0;
    public static double angleGearRatio = 80/12;

    public static double ExtensionP = 0;
    public static double ExtensionI = 0;
    public static double ExtensionD = 0;
    public static double ExtensionS = 0;
    public static double ExtensionV = 0;
    public static double ExtensionA = 0;
    public static double ExtensionG = 0;
    public static double ExtensionSoftLimitLow = 0;
    public static double ExtensionSoftLimitHigh = 0;
    public static double ExtensionGearRatio = 80/10;
}
