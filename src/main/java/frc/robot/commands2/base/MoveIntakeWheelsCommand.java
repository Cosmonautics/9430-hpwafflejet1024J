package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntakeWheelsCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final double m_speed;
    private final boolean m_pressed;

    public MoveIntakeWheelsCommand(IntakeSubsystem intakeSubsystem, double speed, boolean pressed) {
        m_intakeSubsystem = intakeSubsystem;
        m_speed = speed;
        m_pressed = pressed;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_intakeSubsystem.intakePickUp(m_pressed, m_speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
