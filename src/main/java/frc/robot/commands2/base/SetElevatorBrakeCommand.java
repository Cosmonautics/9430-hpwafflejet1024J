package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorBrakeCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    public SetElevatorBrakeCommand(ElevatorSubsystem elevatorSubsystem, boolean brake) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setToBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
