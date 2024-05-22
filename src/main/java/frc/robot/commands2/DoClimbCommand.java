package frc.robot.commands2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands2.base.MoveElevatorCommand;
import frc.robot.commands2.base.SetElevatorBrakeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.PositionConstants.*;

public class DoClimbCommand extends SequentialCommandGroup {
    public DoClimbCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new MoveElevatorCommand(elevatorSubsystem, kElevatorClimbPosition, true),
            new WaitCommand(2.5),
            new SetElevatorBrakeCommand(elevatorSubsystem, true),
            new DoTrapScoreCommand(shooterSubsystem)
        );
    }
}
