// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = 0.6731; // meters
    public static final double kWheelBase = 0.6731; // meters
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 3;
  }

  public static final class ShooterConstants {
    public static final int kShooterLeftCanId = 10;
    public static final int kShooterRightCanId = 9;
    public static final int kShooterPivotCanId = 17;
    public static final int kShooterFeederCanId = 18;
    public static final double kShooterForwardSoftLimit = 0.955;
    public static final double kShooterReverseSoftLimit = 0.158;

    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final class ElevatorConstants {
    public static final double kElevatorForwardSoftLimit = 0.989;
    public static final double kElevatorReverseSoftLimit = 0.045;
    public static final int kElevatorLeftCanId = 13;
    public static final int kElevatorRightCanId = 14;

    public static final double kElevatorUpperSoftLimit = 0.969;
    public static final double kElevatorGearRatio = 1.0;
    public static final double kElevatorDrumDiameterInches = 1.214;
    public static final double kElevatorEncoderTicksPerRevolution = 42.0;
    public static final double kElevatorInchesPerTick = (kElevatorDrumDiameterInches * Math.PI)
        / (kElevatorEncoderTicksPerRevolution * kElevatorGearRatio);

    public static final double kP = 5.0;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kTriggerDeadband = 0.05;
    public static final double kPullyDiameter = 1.214;
    public static final int kElevatorEncoderResolution = 8192; // TODO: 8192 bb
    public static final double kElevatorSetpointInches = 12.0; // Placeholder constant position
    public static final double kGearBoxScale = 0.2045;
    public static final double kPositionToleranceInches = 1.0 / 2.54;
    public static final double kEncoderUnitsPerInch = 1 / kElevatorInchesPerTick;
  }

  public static final class IntakeConstants {
    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int kIntakeLeftCanId = 11;
    // public static final int kIntakeRightCanId = 12;
    public static final int kIntakePivotCanId = 12;

    public static final double kIntakeForwardSoftLimit = 0.275;
    public static final double kIntakeReverseSoftLimit = 0.005;
  }

  public static final class ConveyorConstants {
    public static final int kConveyorCanId = 16;
    public static final int kLimitSwitchChannel = 0; // Update this with the actual channel
  }

  public static final class ModuleConstants {
    // Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T. This changes the drive speed of the module (a pinion gear with more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60; // NEO free speed is 5676 RPM
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class PositionConstants {
    // Elevator Position Constants
    public static final double kElevatorTransitPosition = 0.345;
    public static final double kElevatorShooterPosition = 0.108;
    public static final double kElevatorAMPPosition = 0.197;
    public static final double kElevatorClimb1Position = 0.062;
    public static final double kElevatorClimb2Position = 0.108;
    public static final double kElevatorClimbPosition = 0.930;
    public static final double kElevatorSourceIntakePosition = 0.460;

    // Shooter Position Constants
    public static final double kShooterTransitPosition = 0.240;
    public static final double kShooterShooterPosition = 0.910;
    public static final double kShooterPreShooterPosition = 0.935;
    public static final double kShooterAMPPosition = 0.875;
    public static final double kShooterIntakeSourcePosition = 0.884;
    public static final double kShooterClimb1Position = 0.423; // zero this is intended

    // Intake Position Constants
    public static final double kIntakeTransitPosition = 0.026;
    public static final double kIntakeFloorPosition = 0.308;
    public static final double kIntakeClimb1Position = 0.026;
  }
}
