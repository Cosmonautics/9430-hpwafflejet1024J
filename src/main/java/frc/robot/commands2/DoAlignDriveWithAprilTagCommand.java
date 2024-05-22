package frc.robot.commands2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DoAlignDriveWithAprilTagCommand extends Command {
    private final DriveSubsystem m_drive;
    private final LimelightSubsystem m_limelight;
    private double m_targetOffsetAngleHorizontal;
    private boolean m_isAligned;

    public DoAlignDriveWithAprilTagCommand(DriveSubsystem drive, LimelightSubsystem limelight) {
        m_drive = drive;
        m_limelight = limelight;
        m_targetOffsetAngleHorizontal = 0.0;
        m_isAligned = false;
        addRequirements(m_drive, m_limelight);
    }

    @Override
    public void initialize() {
        m_limelight.setLEDOn();
        m_isAligned = false;
    }

    @Override
    public void execute() {
        m_drive.drive(0, 0, 0, false, false);
        if (m_limelight.hasTarget()) {
            m_targetOffsetAngleHorizontal = m_limelight.getTargetX();
            double rotationSpeed = calculateRotationSpeed(m_targetOffsetAngleHorizontal);
            m_drive.drive(0, 0, -rotationSpeed, false, false);
        } else {
            m_drive.drive(0, 0, 0, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, false, false);
        m_limelight.setLEDOff();
    }

    @Override
    public boolean isFinished() {
        return m_isAligned;
    }

    private double calculateRotationSpeed(double targetOffsetAngle) {
        final double Kp = 0.03;
        double controlEffort = Kp * targetOffsetAngle;
        final double maxRotationSpeed = 0.5;
        controlEffort = Math.max(-maxRotationSpeed, Math.min(controlEffort, maxRotationSpeed));
        final double alignmentThreshold = 2.0;
        m_isAligned = Math.abs(targetOffsetAngle) < alignmentThreshold;
        return controlEffort;
    }
}
