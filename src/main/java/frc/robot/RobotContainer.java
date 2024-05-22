package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DoNoteEjectActionCommand;
import frc.robot.commands.DoNoteIntakeActionCommand;
import frc.robot.commands.DoSourceIntakeActionCommand;
import frc.robot.commands.DoSpeakerScoreActionCommand;
import frc.robot.commands.MoveToAMPSpeakerScorePositionCommand;
import frc.robot.commands.MoveToFloorIntakePositionCommand;
import frc.robot.commands.MoveToTransitPositionCommand;
import frc.robot.commands.StopNoteIntakeEjectActionCommand;
import frc.robot.commands.StopSourceIntakeActionCommand;
import frc.robot.commands2.DoAMPScoreCommand;
import frc.robot.commands2.DoClimb1Command;
import frc.robot.commands2.DoClimbCommand;
import frc.robot.commands2.DoSpeakerScoreAutoCommand;
import frc.robot.commands2.DoSpeakerScoreCommand;
import frc.robot.commands2.DoSpeakerScoreManualCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.ControllerUtils;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands2.autos.Autos;
import static frc.robot.Constants.OIConstants;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final LimelightSubsystem m_limelight = new LimelightSubsystem();
    
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    private Command m_manualShoot;
    private Command m_getAndShootFirstThreeAuto;
    private Command m_threeNoteAuto;
    private Command m_centerlineThreeNote;
    private Command m_shootAndScurryNoNote;
    private Command m_shootAndScurryNote;
    private Command m_oneNoteAutoOnSteroids;
    private boolean isClimb1 = false;
    private boolean isClimb2 = false;

    public RobotContainer() {
        // Register named commands
        ConfigureNamedCommands();
        
        // Configure the button bindings
        ConfigureButtonBindings();
        
        // Set up default drive command
        m_shooter.setDefaultCommand(new RunCommand(() -> {
            double rightTriggerValue = m_operatorController.getRightTriggerAxis();
            if (rightTriggerValue > 0.75 && m_manualShoot != null && !m_manualShoot.isScheduled()) {
                m_manualShoot = new DoSpeakerScoreManualCommand(m_elevator, m_shooter);
                m_manualShoot.schedule();
            }
        }, m_shooter));
        
        m_drive.setDefaultCommand(new RunCommand(() -> {
            m_drive.drive(
                -applyDeadband(m_driverController.getLeftY()),
                -applyDeadband(m_driverController.getLeftX()),
                -applyDeadband(m_driverController.getRightX()),
                true, true
            );
        }, m_drive));
        
        // Initialize autonomous commands
        m_getAndShootFirstThreeAuto = Autos.getAndShootFirstThree(m_elevator, m_shooter, m_drive, m_limelight);
        m_threeNoteAuto = Autos.threeNoteAuto(m_elevator, m_shooter, m_drive, m_limelight);
        m_centerlineThreeNote = Autos.centerlineThreeNote(m_elevator, m_shooter, m_drive, m_limelight);
        m_shootAndScurryNoNote = Autos.shootAndScurryNoNote(m_elevator, m_shooter, m_drive, m_limelight);
        m_shootAndScurryNote = Autos.shootAndScurryNote(m_elevator, m_shooter, m_drive, m_limelight);
        m_oneNoteAutoOnSteroids = Autos.oneNoteAutoOnSteroids(m_elevator, m_shooter, m_drive, m_limelight);
        
        ConfigureAutoChooser();
    }

    private void ConfigureNamedCommands() {
        // Register commands with the PathPlanner
        NamedCommands.registerCommand("MoveToFloorIntakePositionCommand", new MoveToFloorIntakePositionCommand(m_elevator, m_shooter, m_intake));
        NamedCommands.registerCommand("MoveToAMPSpeakerScorePositionCommand", new MoveToAMPSpeakerScorePositionCommand(m_elevator, m_shooter));
        NamedCommands.registerCommand("MoveToTransitPositionCommand", new MoveToTransitPositionCommand(m_elevator, m_shooter, m_intake));
        NamedCommands.registerCommand("DoClimb1Command", new DoClimb1Command(m_elevator, m_shooter, m_intake));
        NamedCommands.registerCommand("DoSpeakerScoreAutoCommand", new DoSpeakerScoreAutoCommand(m_elevator, m_shooter, m_limelight));
        NamedCommands.registerCommand("DoNoteEjectActionCommand", new DoNoteEjectActionCommand(m_conveyor, m_shooter, m_intake));
        NamedCommands.registerCommand("DoNoteIntakeActionCommand", new DoNoteIntakeActionCommand(m_conveyor, m_shooter, m_intake));
        NamedCommands.registerCommand("DoSourceIntakeActionCommand", new DoSourceIntakeActionCommand(m_elevator, m_shooter));
        NamedCommands.registerCommand("StopNoteIntakeEjectActionCommand", new StopNoteIntakeEjectActionCommand(m_conveyor, m_shooter, m_intake));
        NamedCommands.registerCommand("StopSourceIntakeActionCommand", new StopSourceIntakeActionCommand(m_elevator, m_shooter));
        NamedCommands.registerCommand("DoAMPScoreCommand", new DoAMPScoreCommand(m_elevator, m_shooter));
        NamedCommands.registerCommand("DoSpeakerScoreActionCommand", new DoSpeakerScoreActionCommand(m_elevator, m_shooter));
        NamedCommands.registerCommand("DoClimbCommand", new DoClimbCommand(m_elevator, m_shooter));
    }

    private void ConfigureButtonBindings() {
        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(() -> {
                m_drive.zeroHeading();
                ControllerUtils.vibrateController(m_driverController, 0.8, 0.3);
            }, m_drive));
        
        new POVButton(m_operatorController, 270)
            .onTrue(new MoveToFloorIntakePositionCommand(m_elevator, m_shooter, m_intake));
        
        new POVButton(m_operatorController, 90)
            .onTrue(new MoveToAMPSpeakerScorePositionCommand(m_elevator, m_shooter));
        
        new POVButton(m_operatorController, 180)
            .onTrue(new MoveToTransitPositionCommand(m_elevator, m_shooter, m_intake));
        
        new POVButton(m_operatorController, 0)
            .onTrue(new InstantCommand(() -> {
                if (!isClimb2) {
                    isClimb1 = true;
                    new DoClimb1Command(m_elevator, m_shooter, m_intake).schedule();
                } 
            }, m_elevator, m_shooter, m_intake));
        
        new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
            .onTrue(new DoNoteIntakeActionCommand(m_conveyor, m_shooter, m_intake))
            .onFalse(new StopNoteIntakeEjectActionCommand(m_conveyor, m_shooter, m_intake));
        
        new JoystickButton(m_operatorController, XboxController.Button.kB.value)
            .onTrue(new DoAMPScoreCommand(m_elevator, m_shooter));
        
        new JoystickButton(m_operatorController, XboxController.Button.kA.value)
            .onTrue(new DoNoteEjectActionCommand(m_conveyor, m_shooter, m_intake))
            .onFalse(new StopNoteIntakeEjectActionCommand(m_conveyor, m_shooter, m_intake));
        
        new JoystickButton(m_operatorController, XboxController.Button.kX.value)
            .onTrue(new DoSourceIntakeActionCommand(m_elevator, m_shooter))
            .onFalse(new StopSourceIntakeActionCommand(m_elevator, m_shooter));
        
        new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
            .onTrue(new InstantCommand(() -> {
                SmartDashboard.putNumber("Limelight Distance", m_limelight.calculateDistanceToTarget(true));
                new DoSpeakerScoreCommand(m_elevator, m_shooter, m_drive, m_limelight).schedule();
            }, m_elevator, m_shooter, m_drive, m_limelight));
        
        new JoystickButton(m_operatorController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(() -> {
                if (isClimb1) {
                    isClimb1 = false;
                    new DoClimbCommand(m_elevator, m_shooter).schedule();
                }
            }, m_elevator, m_shooter));
    }

    private void ConfigureAutoChooser() {
        try {
            m_chooser.addOption("Do Nothing", new InstantCommand(() -> {}));
            m_chooser.addOption("Shoot Note and Do Nothing", new DoSpeakerScoreCommand(m_elevator, m_shooter, m_drive, m_limelight));
            m_chooser.addOption("Get First 2 Notes and Shoot", m_getAndShootFirstThreeAuto);
            m_chooser.addOption("Get First Note and Shoot Far Note", m_oneNoteAutoOnSteroids);
            m_chooser.addOption("Get And Shoot Center 2", m_oneNoteAutoOnSteroids);
            m_chooser.addOption("Three Notes Starting with center note", m_centerlineThreeNote);
            m_chooser.addOption("Shoot And Scurry Note", m_shootAndScurryNote);
            m_chooser.addOption("Shoot And Scurry No Note", m_shootAndScurryNoNote);
            m_chooser.setDefaultOption("Get First 3 Notes and Shoot Close", m_threeNoteAuto);
            
            SmartDashboard.putData("Autonomous Options", m_chooser);
        } catch (Exception ex) {
            String err_msg = "Error Starting Autonomous:  " + ex.getMessage();
            System.out.println(err_msg);
        }
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > OIConstants.kDriveDeadband ? value : 0.0;
    }
}
