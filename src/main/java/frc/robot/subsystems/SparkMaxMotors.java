package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class SparkMaxMotors extends SubsystemBase {
    private final SparkMax m_SparkMaxMotor1;
    private final SparkMax m_SparkMaxMotor2;
    SparkMaxConfig config = new SparkMaxConfig();

    public SparkMaxMotors() {
        m_SparkMaxMotor1 = new SparkMax(MotorConstants.kSparkMaxMotor1CANID, MotorType.kBrushless);
        m_SparkMaxMotor2 = new SparkMax(MotorConstants.kSparkMaxMotor2CANID, MotorType.kBrushless);

        updateMotorSettings(m_SparkMaxMotor1);
        updateMotorSettings(m_SparkMaxMotor2);
    }
    public void updateMotorSettings(SparkMax motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kSparkMaxMotorCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   
    public void setSpeed(double speed) {
        if (speed>MotorConstants.kSparkMaxMotorMaxSpeed)
          speed = MotorConstants.kSparkMaxMotorMaxSpeed;
        m_SparkMaxMotor1.set(speed);
        m_SparkMaxMotor2.set(speed);
    }
}
