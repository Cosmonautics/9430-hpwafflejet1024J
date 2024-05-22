package frc.robot.commands2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands2.base.MoveElevatorCommand;
import frc.robot.commands2.base.PivotIntakeCommand;
import frc.robot.commands2.base.PivotShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.PositionConstants.*;

public class DoClimb1Command extends SequentialCommandGroup {
    public DoClimb1Command(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            new PivotShooterCommand(shooterSubsystem, kShooterPreShooterPosition),
            new WaitCommand(0.5),
            new MoveElevatorCommand(elevatorSubsystem, kElevatorClimb1Position, false),
            new PivotIntakeCommand(intakeSubsystem, kIntakeClimb1Position, true)
        );
    }
}
