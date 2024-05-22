package frc.robot.commands2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands2.base.MoveElevatorCommand;
import frc.robot.commands2.base.MoveShooterFeederWheelsCommand;
import frc.robot.commands2.base.MoveShooterWheelsCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.PositionConstants.*;

public class DoSpeakerScoreCommand extends SequentialCommandGroup {
    public DoSpeakerScoreCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
        addCommands(
            new MoveShooterWheelsCommand(shooterSubsystem, -1.0),
            new DoAlignDriveWithAprilTagCommand(driveSubsystem, limelightSubsystem),
            new DoAlignShooterWithAprilTagCommand(shooterSubsystem, limelightSubsystem, elevatorSubsystem),
            new MoveElevatorCommand(elevatorSubsystem, kElevatorShooterPosition, false),
            new WaitCommand(1.2),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
            new WaitCommand(0.5),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
            new MoveShooterWheelsCommand(shooterSubsystem, 0.0)
        );
    }
}
