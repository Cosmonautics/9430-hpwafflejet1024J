package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DoSpeakerScoreActionCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private boolean cmdFinished;
    private Timer timer;

    public DoSpeakerScoreActionCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_elevatorSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        cmdFinished = false;
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void execute() {
        timer.start();
        m_shooterSubsystem.shootMotors(true, -1.0);

        m_shooterSubsystem.invertMotor(true);
        m_shooterSubsystem.pivotToSetPoint(PositionConstants.kShooterShooterPosition);
        m_elevatorSubsystem.moveToPosition(PositionConstants.kElevatorShooterPosition, false);

        while (!timer.hasElapsed(1.5)) {
            // Wait for 1.5 seconds
        }

        m_shooterSubsystem.moveFeeder(-1.0);

        while (!timer.hasElapsed(2.0)) {
            // Wait for another 0.5 seconds
        }

        m_shooterSubsystem.stopMotors();
        m_shooterSubsystem.moveFeeder(0);
        timer.stop();
        cmdFinished = true;
    }

    @Override
    public boolean isFinished() {
        return cmdFinished;
    }
}
