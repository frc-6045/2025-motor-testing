package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MotorToTest;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotContainer;

public class SparkFlexTest extends Command {
    private final MotorToTest m_IntakeSubsystem;
    private final CommandXboxController controller;
    public SparkFlexTest(MotorToTest intakeSubsystem, CommandXboxController xboxController) {
        m_IntakeSubsystem = intakeSubsystem;
        controller = xboxController;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void execute() {
        double speed = MotorConstants.kSparkFlexMotorMaxSpeed;
        double triggerAxis = controller.getLeftTriggerAxis()-controller.getRightTriggerAxis();
        m_IntakeSubsystem.setSpeed(speed, triggerAxis);
        System.out.println("speed: " + speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setSpeed(0, 0);
    }

}
