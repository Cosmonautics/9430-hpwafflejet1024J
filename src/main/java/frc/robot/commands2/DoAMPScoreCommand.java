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

public class DoAMPScoreCommand extends SequentialCommandGroup {
    public DoAMPScoreCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new PivotShooterCommand(shooterSubsystem, kShooterAMPPosition),
            new MoveElevatorCommand(elevatorSubsystem, kElevatorAMPPosition, false),
            new WaitCommand(1),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
            new MoveShooterWheelsCommand(shooterSubsystem, -0.10),
            new WaitCommand(1.7),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
            new MoveShooterWheelsCommand(shooterSubsystem, 0.0)
        );
    }
}
