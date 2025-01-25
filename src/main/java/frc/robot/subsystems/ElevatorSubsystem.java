package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.SparkFlex;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex m_ElevatorMotor1;
    private final SparkFlex m_ElevatorMotor2;
    private final RelativeEncoder m_RelativeEncoder;
    SparkFlexConfig config = new SparkFlexConfig();

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    m_ElevatorMotor1 = new SparkFlex(MotorConstants.kSparkFlexElevatorMotor1CANID, MotorType.kBrushless);
    m_ElevatorMotor2 = new SparkFlex(MotorConstants.kSparkFlexElevatorMotor2CANID, MotorType.kBrushless);
    updateMotorSettings(m_ElevatorMotor1);
    updateMotorSettings(m_ElevatorMotor2);
    m_RelativeEncoder = m_ElevatorMotor1.getEncoder();
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    if (speed>MotorConstants.kSparkFlexElevatorMotorsMaxSpeed)
      speed = MotorConstants.kSparkFlexElevatorMotorsMaxSpeed;
    m_ElevatorMotor1.set(speed);
    m_ElevatorMotor2.set(-speed);
  }

  public void stopElevatorMotors() {
    m_ElevatorMotor1.stopMotor();
    m_ElevatorMotor2.stopMotor();
  }

  public RelativeEncoder gRelativeEncoder() {
    return m_RelativeEncoder;
  }

  public double getAbsoluteEncoderPosition() {
    return m_RelativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator", getAbsoluteEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
