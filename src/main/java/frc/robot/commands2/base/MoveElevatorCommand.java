package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_position;
    private final boolean m_isClimb;

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, double position, boolean isClimb) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_position = position;
        m_isClimb = isClimb;
        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_elevatorSubsystem.moveToPosition(m_position, m_isClimb);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
