package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterFeederWheelsCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_speed;

    public MoveShooterFeederWheelsCommand(ShooterSubsystem shooterSubsystem, double speed) {
        m_shooterSubsystem = shooterSubsystem;
        m_speed = speed;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shooterSubsystem.moveFeeder(m_speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
