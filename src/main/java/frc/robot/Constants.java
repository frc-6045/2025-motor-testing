// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MotorConstants {
    public static final int kSparkFlexMotor1CANID = 1;
    public static final int kSparkFlexMotor2CANID = 2;
    public static final double kSparkFlexMotorMaxSpeed = 0.5;
    public static final int kSparkFlexMotorCurrentLimit = 40;

    public static final int kSparkMaxMotor1CANID = 3;
    public static final int kSparkMaxMotor2CANID = 4;
    public static final double kSparkMaxMotorMaxSpeed = 0.5;
    public static final int kSparkMaxMotorCurrentLimit = 40;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
    public static double kHomeArmPosition;
  }
}

// maek controler wurk plez