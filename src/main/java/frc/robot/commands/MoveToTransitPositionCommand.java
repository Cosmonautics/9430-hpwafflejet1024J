package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveToTransitPositionCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private boolean cmdFinished;

    public MoveToTransitPositionCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_elevatorSubsystem, m_shooterSubsystem, m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        cmdFinished = true;  // initial state
    }

    @Override
    public void execute() {
        m_shooterSubsystem.invertMotor(true);
        m_shooterSubsystem.stopMotors();
        m_shooterSubsystem.moveFeeder(0.0);
        m_shooterSubsystem.pivotToSetPoint(PositionConstants.kShooterTransitPosition);
        m_intakeSubsystem.pivotToAngle(PositionConstants.kIntakeTransitPosition, false);

        m_elevatorSubsystem.moveToPosition(PositionConstants.kElevatorTransitPosition, false);
        cmdFinished = true;  // set to true if all actions are complete
    }

    @Override
    public boolean isFinished() {
        return cmdFinished;
    }
}
