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
    // arm constants
    public static final int kSparkFlexArmMotorCANID = 9;

    public static final int kSparkFlexArmMotorCurrentLimit = 40;
    public static final double kSparkFlexArmMotorSpeed = .2;
    public static final double kSparkFlexArmMotorMaxSpeed = 0.5;

    //public static final double armEncoderHorizontalOffset = 0.1649185041229626;

    // elevator constants
    public static final int kSparkFlexElevatorMotor1CANID = 10;
    public static final int kSparkFlexElevatorMotor2CANID = 11;

    public static final int kSparkFlexElevatorMotorsCurrentLimit = 40;
    public static final double kSparkFlexElevatorMotorsSpeed = .29;
    public static final double kSparkFlexElevatorMotorsMaxSpeed = .4;

    // intake constants
    public static final int kIntakeMotor1CANID = 12;
    public static final int kIntakeMotor2CANID = 13;

    public static final int kIntakeMotorsCurrentLimit = 40;
    public static final double kIntakeMotorsSpeed = .5;
    public static final double kIntakeMotorsMaxSpeed = .6;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
    public static double kHomeArmPosition;
  }

  public static class PositionConstants {
    // Home setpoint
    public static final double kHomeArmPosition = 0;
    public static final double kHomeElevatorPosition = 0;

    // Human player intake setpoint
    public static final double kHumanArmPosition = 0;
    public static final double kHumanElevatorPosition = 0;

    // Setpoint 1
    public static final double kSetpoint1ArmPosition = 0;
    public static final double kSetpoint1ElevatorPosition = 0;
  }
}

// maek controler wurk plez