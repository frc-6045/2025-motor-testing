package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class StopPIDArmCommand extends InstantCommand {
    private final ArmSubsystem m_ArmMotor;
    public StopPIDArmCommand(ArmSubsystem armMotor) {
        m_ArmMotor=armMotor;
        addRequirements(m_ArmMotor);
    }

    @Override
    public void initialize() {m_ArmMotor.stopArmMotor();}
}
