package frc.robot.commands2.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands2.DoSpeakerScoreManualCommand;
import frc.robot.commands2.DoSpeakerScoreCommand;

public class Autos {

    public static Command getAndShootFirstThree(ElevatorSubsystem elevatorSubsystem,
                                                ShooterSubsystem shooterSubsystem,
                                                DriveSubsystem driveSubsystem,
                                                LimelightSubsystem limelightSubsystem) {
        return Commands.sequence(
            new DoSpeakerScoreManualCommand(elevatorSubsystem, shooterSubsystem),
            Commands.waitSeconds(0.2),
            new PathPlannerAuto("GetAndShootFirstThree")
        );
    }

    public static Command threeNoteAuto(ElevatorSubsystem elevatorSubsystem,
                                        ShooterSubsystem shooterSubsystem,
                                        DriveSubsystem driveSubsystem,
                                        LimelightSubsystem limelightSubsystem) {
        return Commands.sequence(
            new DoSpeakerScoreManualCommand(elevatorSubsystem, shooterSubsystem),
            Commands.waitSeconds(0.2),
            new PathPlannerAuto("ThreeNoteAuto"),
            new DoSpeakerScoreCommand(elevatorSubsystem, shooterSubsystem, driveSubsystem, limelightSubsystem)
        );
    }

    public static Command oneNoteAutoOnSteroids(ElevatorSubsystem elevatorSubsystem,
                                                ShooterSubsystem shooterSubsystem,
                                                DriveSubsystem driveSubsystem,
                                                LimelightSubsystem limelightSubsystem) {
        return Commands.sequence(
            new DoSpeakerScoreManualCommand(elevatorSubsystem, shooterSubsystem),
            Commands.waitSeconds(0.2),
            new PathPlannerAuto("OneNoteAutoOnSteroids"),
            new DoSpeakerScoreCommand(elevatorSubsystem, shooterSubsystem, driveSubsystem, limelightSubsystem)
        );
    }

    public static Command centerlineThreeNote(ElevatorSubsystem elevatorSubsystem,
                                              ShooterSubsystem shooterSubsystem,
                                              DriveSubsystem driveSubsystem,
                                              LimelightSubsystem limelightSubsystem) {
        return Commands.sequence(
            new DoSpeakerScoreManualCommand(elevatorSubsystem, shooterSubsystem),
            new PathPlannerAuto("centerlineThreeNote"),
            new DoSpeakerScoreCommand(elevatorSubsystem, shooterSubsystem, driveSubsystem, limelightSubsystem)
        );
    }

    public static Command shootAndScurryNoNote(ElevatorSubsystem elevatorSubsystem,
                                               ShooterSubsystem shooterSubsystem,
                                               DriveSubsystem driveSubsystem,
                                               LimelightSubsystem limelightSubsystem) {
        return Commands.sequence(
            new DoSpeakerScoreManualCommand(elevatorSubsystem, shooterSubsystem),
            Commands.waitSeconds(10),
            new PathPlannerAuto("ShootAndScurryNoNote")
        );
    }

    public static Command shootAndScurryNote(ElevatorSubsystem elevatorSubsystem,
                                             ShooterSubsystem shooterSubsystem,
                                             DriveSubsystem driveSubsystem,
                                             LimelightSubsystem limelightSubsystem) {
        return Commands.sequence(
            new DoSpeakerScoreManualCommand(elevatorSubsystem, shooterSubsystem),
            Commands.waitSeconds(10),
            new PathPlannerAuto("ShootAndScurryNote")
        );
    }
}
