package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DoSourceIntakeActionCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public DoSourceIntakeActionCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_elevatorSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shooterSubsystem.invertMotor(true);
        m_shooterSubsystem.pivotToSetPoint(PositionConstants.kShooterIntakeSourcePosition);
        m_elevatorSubsystem.moveToPosition(PositionConstants.kElevatorSourceIntakePosition, false);
        m_shooterSubsystem.shooterPickUpNote(true, 0.30);
        m_shooterSubsystem.moveFeeder(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
