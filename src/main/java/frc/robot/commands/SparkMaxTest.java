package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SparkMaxMotors;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotContainer;

public class SparkMaxTest extends Command {
    private final SparkMaxMotors m_IntakeSubsystem;
    private final boolean direction;
    public SparkMaxTest(SparkMaxMotors motorSubsystem, boolean direction) {
        m_IntakeSubsystem = motorSubsystem;
        this.direction = direction;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute() {
        double speed = MotorConstants.kSparkMaxMotorMaxSpeed;
        m_IntakeSubsystem.setSpeed(speed);
        System.out.println("speed: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setSpeed(0);
    }

}
