package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax m_ElevatorMotorLeft;
    private final CANSparkMax m_ElevatorMotorRight;
    private final SparkPIDController m_ElevatorPIDController;
    private final RelativeEncoder m_ElevatorThroughBoreEncoder;

    private double currentPositionInches;
    private double targetPositionInches;
    private boolean manualOverride;

    public ElevatorSubsystem() {
        m_ElevatorMotorLeft = new CANSparkMax(ElevatorConstants.kElevatorLeftCanId, MotorType.kBrushless);
        m_ElevatorMotorRight = new CANSparkMax(ElevatorConstants.kElevatorRightCanId, MotorType.kBrushless);
        m_ElevatorPIDController = m_ElevatorMotorRight.getPIDController();
        m_ElevatorThroughBoreEncoder = m_ElevatorMotorRight.getEncoder();
        ConfigureMotors();
    }

    private void ConfigureMotors() {
        m_ElevatorMotorLeft.restoreFactoryDefaults();
        m_ElevatorMotorRight.restoreFactoryDefaults();
        m_ElevatorMotorLeft.follow(m_ElevatorMotorRight, true);

        m_ElevatorMotorLeft.setSmartCurrentLimit(40);
        m_ElevatorMotorRight.setSmartCurrentLimit(40);

        try {
            m_ElevatorPIDController.setFeedbackDevice(m_ElevatorThroughBoreEncoder);
        } catch (Exception ex) {
            System.out.println("Error Setting feedback device: " + ex.getMessage());
        }
        m_ElevatorPIDController.setP(ElevatorConstants.kP);
        m_ElevatorPIDController.setI(ElevatorConstants.kI);
        m_ElevatorPIDController.setD(ElevatorConstants.kD);
        m_ElevatorPIDController.setOutputRange(-1.0, 1.0);
        m_ElevatorMotorRight.setSoftLimit(SoftLimitDirection.kForward, (float) ElevatorConstants.kElevatorForwardSoftLimit);
        m_ElevatorMotorRight.setSoftLimit(SoftLimitDirection.kReverse, (float) ElevatorConstants.kElevatorReverseSoftLimit);
        m_ElevatorMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_ElevatorMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void moveToPosition(double positionRotations, boolean isClimb) {
        if (isClimb) {
            m_ElevatorPIDController.setP(2.0);
        } else {
            m_ElevatorPIDController.setP(ElevatorConstants.kP);
        }
        targetPositionInches = rotationsToInches(positionRotations);
        m_ElevatorPIDController.setReference(positionRotations, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        updatePosition();
    }

    public void setToBrakeMode() {
        m_ElevatorMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_ElevatorMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    private void updatePosition() {
        double currentPositionRotations = m_ElevatorThroughBoreEncoder.getPosition();
        currentPositionInches = rotationsToInches(currentPositionRotations);
    }

    public double calculateTargetHeight(double targetRevolutions) {
        double currentRevolutions = m_ElevatorThroughBoreEncoder.getPosition() / ElevatorConstants.kElevatorEncoderResolution;
        double revolutionDifference = targetRevolutions - currentRevolutions;
        return revolutionDifference;
    }

    public double inchesToRotations(double inches) {
        return (ElevatorConstants.kGearBoxScale * inches) / (ElevatorConstants.kPullyDiameter * Math.PI);
    }

    public double rotationsToInches(double rotations) {
        return (rotations * ElevatorConstants.kPullyDiameter * Math.PI) / ElevatorConstants.kGearBoxScale;
    }

    public boolean atTargetPosition() {
        return Math.abs(currentPositionInches - targetPositionInches) <= ElevatorConstants.kPositionToleranceInches;
    }

    public boolean toggleManualOverride() {
        manualOverride = !manualOverride;
        return manualOverride;
    }

    public void manualMove(double speed) {
        if (manualOverride) {
            double currentElevatorPosition = m_ElevatorThroughBoreEncoder.getPosition();
            double speedFactor = 0.01;
            double targetPositionRotations = currentElevatorPosition + (speed * speedFactor);
            targetPositionInches = rotationsToInches(targetPositionRotations);
            m_ElevatorPIDController.setReference(targetPositionRotations, CANSparkMax.ControlType.kPosition);
        }
    }
}
