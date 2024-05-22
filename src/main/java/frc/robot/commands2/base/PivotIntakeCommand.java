package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final double m_position;
    private final boolean m_down;

    public PivotIntakeCommand(IntakeSubsystem intakeSubsystem, double position, boolean down) {
        m_intakeSubsystem = intakeSubsystem;
        m_position = position;
        m_down = down;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_intakeSubsystem.pivotToAngle(m_position, m_down);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
