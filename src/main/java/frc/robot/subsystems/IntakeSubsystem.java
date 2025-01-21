package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_IntakeMotor1;
    private final SparkFlex m_IntakeMotor2;
    SparkFlexConfig config = new SparkFlexConfig();

    public IntakeSubsystem() {
        m_IntakeMotor1 = new SparkFlex(MotorConstants.kIntakeMotor1CANID, MotorType.kBrushless);
        m_IntakeMotor2 = new SparkFlex(MotorConstants.kIntakeMotor2CANID, MotorType.kBrushless);

        //updateMotorSettings(m_IntakeMotor1);
        //updateMotorSettings(m_IntakeMotor2);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
        //motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   
    public void setSpeed(double speed, int runMotor1, double runMotor2) {
        if (speed>MotorConstants.kSparkFlexElevatorMotorsMaxSpeed)
          speed = MotorConstants.kSparkFlexElevatorMotorsMaxSpeed;
        m_IntakeMotor1.set(speed*runMotor1);
        m_IntakeMotor2.set(speed*runMotor2);
    }
}
