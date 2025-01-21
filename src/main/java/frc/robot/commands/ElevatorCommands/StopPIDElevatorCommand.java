package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class StopPIDElevatorCommand extends InstantCommand {
    private final ElevatorSubsystem m_ElevatorSubsystem;
    public StopPIDElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        m_ElevatorSubsystem = elevatorSubsystem;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void initialize() {m_ElevatorSubsystem.stopElevatorMotors();}
}
