package frc.robot.commands2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands2.base.MoveElevatorCommand;
import frc.robot.commands2.base.MoveShooterFeederWheelsCommand;
import frc.robot.commands2.base.MoveShooterWheelsCommand;
import frc.robot.commands2.base.PivotShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.PositionConstants.*;

public class DoSpeakerScoreManualCommand extends SequentialCommandGroup {
    public DoSpeakerScoreManualCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new MoveShooterWheelsCommand(shooterSubsystem, -1.0),
            new PivotShooterCommand(shooterSubsystem, 0.85),
            new WaitCommand(0.5),
            new MoveElevatorCommand(elevatorSubsystem, kElevatorShooterPosition, false),
            new WaitCommand(0.5),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
            new WaitCommand(0.5),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
            new MoveShooterWheelsCommand(shooterSubsystem, 0.0)
        );
    }
}
