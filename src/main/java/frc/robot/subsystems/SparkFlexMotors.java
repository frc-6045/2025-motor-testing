package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class SparkFlexMotors extends SubsystemBase {
    private final SparkFlex m_SparkFlexMotor1;
    private final SparkFlex m_SparkFlexMotor2;
    SparkFlexConfig config = new SparkFlexConfig();

    public SparkFlexMotors() {
        m_SparkFlexMotor1 = new SparkFlex(MotorConstants.kSparkFlexMotor1CANID, MotorType.kBrushless);
        m_SparkFlexMotor2 = new SparkFlex(MotorConstants.kSparkFlexMotor2CANID, MotorType.kBrushless);

        updateMotorSettings(m_SparkFlexMotor1);
        updateMotorSettings(m_SparkFlexMotor2);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kSparkFlexMotorCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   
    public void setSpeed(double speed, double runMotors) {
        if (speed>MotorConstants.kSparkFlexMotorMaxSpeed)
          speed = MotorConstants.kSparkFlexMotorMaxSpeed;
        m_SparkFlexMotor1.set(speed*runMotors);
        m_SparkFlexMotor2.set(speed*runMotors);
    }
}
