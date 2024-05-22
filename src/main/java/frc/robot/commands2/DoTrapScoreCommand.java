package frc.robot.commands2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands2.base.MoveShooterFeederWheelsCommand;
import frc.robot.commands2.base.MoveShooterWheelsCommand;
import frc.robot.commands2.base.PivotShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.PositionConstants.*;

public class DoTrapScoreCommand extends SequentialCommandGroup {
    public DoTrapScoreCommand(ShooterSubsystem shooterSubsystem) {
        addCommands(
            new PivotShooterCommand(shooterSubsystem, kShooterAMPPosition - 0.04),
            new WaitCommand(0.8),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
            new MoveShooterWheelsCommand(shooterSubsystem, -0.20),
            new WaitCommand(1.7),
            new MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
            new MoveShooterWheelsCommand(shooterSubsystem, 0.0)
        );
    }
}
