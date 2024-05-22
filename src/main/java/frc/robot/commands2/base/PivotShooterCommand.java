package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PivotShooterCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_position;

    public PivotShooterCommand(ShooterSubsystem shooterSubsystem, double position) {
        m_shooterSubsystem = shooterSubsystem;
        m_position = position;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shooterSubsystem.pivotToSetPoint(m_position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
