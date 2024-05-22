package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopSourceIntakeActionCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public StopSourceIntakeActionCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_elevatorSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shooterSubsystem.stopMotors();
        m_shooterSubsystem.moveFeeder(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
