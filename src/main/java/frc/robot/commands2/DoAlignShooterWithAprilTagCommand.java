package frc.robot.commands2;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class DoAlignShooterWithAprilTagCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final LimelightSubsystem m_limelight;
    private final ElevatorSubsystem m_elevator;
    private boolean m_isAligned;

    public DoAlignShooterWithAprilTagCommand(ShooterSubsystem shooter, LimelightSubsystem limelight, ElevatorSubsystem elevator) {
        m_shooter = shooter;
        m_limelight = limelight;
        m_elevator = elevator;
        m_isAligned = false;
        addRequirements(m_shooter, m_limelight, m_elevator);
    }

    @Override
    public void initialize() {
        m_limelight.setLEDOn();
        m_isAligned = false;
    }

    @Override
    public void execute() {
        if (m_limelight.hasTarget()) {
            m_shooter.setAngleBasedOnDistance(
                m_limelight.calculateDistanceToTarget(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            );
        }
        m_isAligned = true;
    }

    @Override
    public void end(boolean interrupted) {
        m_limelight.setLEDOff();
    }

    @Override
    public boolean isFinished() {
        return m_isAligned;
    }
}
