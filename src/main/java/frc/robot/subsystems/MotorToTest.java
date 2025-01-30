package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class MotorToTest extends SubsystemBase {
    private final SparkFlex m_SparkFlexMotor;
    SparkFlexConfig config = new SparkFlexConfig();

    public MotorToTest() {
        m_SparkFlexMotor = new SparkFlex(MotorConstants.kSparkFlexMotorCANID, MotorType.kBrushless);

        updateMotorSettings(m_SparkFlexMotor);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kSparkFlexMotorCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   
    public void setSpeed(double speed, double runMotor1) {
        if (speed>MotorConstants.kSparkFlexElevatorMotorsMaxSpeed)
          speed = MotorConstants.kSparkFlexElevatorMotorsMaxSpeed;
        m_SparkFlexMotor.set(speed*runMotor1);
    }
}
