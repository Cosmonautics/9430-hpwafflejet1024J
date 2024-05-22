package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopNoteIntakeEjectActionCommand extends Command {
    private final ConveyorSubsystem m_conveyorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private boolean cmdFinished;

    public StopNoteIntakeEjectActionCommand(ConveyorSubsystem conveyorSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_conveyorSubsystem = conveyorSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_conveyorSubsystem, m_shooterSubsystem, m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        cmdFinished = false;
    }

    @Override
    public void execute() {
        m_intakeSubsystem.stopMotors();
        m_conveyorSubsystem.stop();
        m_shooterSubsystem.stopMotors();
        m_shooterSubsystem.moveFeeder(0);
        cmdFinished = true;
    }

    @Override
    public boolean isFinished() {
        return cmdFinished;
    }
}
