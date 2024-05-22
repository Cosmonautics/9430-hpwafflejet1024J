package frc.robot.commands2.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class MoveConveyorCommand extends Command {
    private final ConveyorSubsystem m_conveyorSubsystem;
    private final double m_speed;

    public MoveConveyorCommand(ConveyorSubsystem conveyorSubsystem, double speed) {
        m_conveyorSubsystem = conveyorSubsystem;
        m_speed = speed;
        addRequirements(m_conveyorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_conveyorSubsystem.move(m_speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
