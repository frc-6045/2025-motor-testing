// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;



import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkMax m_turningSparkMax;

  private SparkFlexConfig m_DrivingConfig = new SparkFlexConfig();
  private SparkMaxConfig m_TurningConfig = new SparkMaxConfig();

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  //private final SparkPIDController m_drivingPIDController;
  //private final SparkPIDController m_turningPIDController;
  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  private final SimpleMotorFeedforward m_SimpleMotorFeedforward;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean isInverted) {
    m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    //m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    //m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    //m_drivingPIDController = m_drivingSparkFlex.getPIDController();
    //m_turningPIDController = m_turningSparkMax.getPIDController();
    //m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    //m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    /*m_DrivingConfig
      .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
      .inverted(isInverted)
      .idleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_DrivingConfig.encoder
      .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    m_DrivingConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
      .pidf(ModuleConstants.kDrivingP,ModuleConstants.kDrivingI,ModuleConstants.kDrivingD,ModuleConstants.kDrivingFF);
    m_DrivingConfig.signals
      .primaryEncoderPositionPeriodMs(5);
    
    m_TurningConfig
      .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
      .inverted(ModuleConstants.kTurningEncoderInverted)
      .idleMode(ModuleConstants.kTurningMotorIdleMode);
    m_TurningConfig.encoder
      .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
      .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    m_TurningConfig.closedLoop
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
      .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
      .pidf(ModuleConstants.kTurningP,ModuleConstants.kTurningI,ModuleConstants.kTurningD,ModuleConstants.kTurningFF);
    m_TurningConfig.signals
      .primaryEncoderPositionPeriodMs(5);*/

    m_drivingSparkFlex.configure(m_DrivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_TurningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    //Characterization Constants
    m_SimpleMotorFeedforward = new SimpleMotorFeedforward(0.198228, .342548); //ks: 0.1902 kv: .346228
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    //m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, m_SimpleMotorFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond));
    //m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, m_SimpleMotorFeedforward.calculate(correctedDesiredState.speedMetersPerSecond));
    m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getEncoderCounts()
  {
    //return m_drivingEncoder.getPositionConversionFactor() * m_drivingEncoder.getPosition();
    return m_drivingSparkFlex.configAccessor.encoder.getPositionConversionFactor() 
        *  m_drivingEncoder.getPosition();
  }

  public void setDriveVoltage(double voltage)
  {
    m_drivingSparkFlex.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage)
  {
    m_turningSparkMax.setVoltage(voltage);
  }


public void runCharacterization(double volts, double offset){ 
  //reminder, this is different in AdvantageKit
  m_turningPIDController.setReference(offset, ControlType.kPosition);
  setDriveVoltage(volts);
}
public double getCharacterizationVelocity(){
  return m_drivingEncoder.getVelocity() * 2 * Math.PI; // rads per sec 
}


}
