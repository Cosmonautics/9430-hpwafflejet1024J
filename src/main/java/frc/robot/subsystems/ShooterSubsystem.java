package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.util.ArrayList;
import java.util.List;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_pivotMotor;
    private final CANSparkMax m_shooterMotorLeft;
    private final CANSparkMax m_shooterMotorRight;
    private final CANSparkMax m_shooterFeeder;
    private final SparkPIDController m_pivotPIDController;
    private final RelativeEncoder m_pivotEncoder;

    private double m_targetSetpointRotations;
    private boolean manualOverride;
    private List<DistanceAnglePair> distanceAngleLookup;

    public ShooterSubsystem() {
        m_pivotMotor = new CANSparkMax(ShooterConstants.kShooterPivotCanId, MotorType.kBrushless);
        m_shooterMotorLeft = new CANSparkMax(ShooterConstants.kShooterLeftCanId, MotorType.kBrushless);
        m_shooterMotorRight = new CANSparkMax(ShooterConstants.kShooterRightCanId, MotorType.kBrushless);
        m_shooterFeeder = new CANSparkMax(ShooterConstants.kShooterFeederCanId, MotorType.kBrushless);
        m_pivotPIDController = m_pivotMotor.getPIDController();
        m_pivotEncoder = m_pivotMotor.getEncoder();
        configureMotors();
        initializeDistanceAngleLookup();
    }

    private void configureMotors() {
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor.setInverted(true);
        m_shooterFeeder.setInverted(true);
        m_pivotMotor.setSmartCurrentLimit(20);
        m_shooterFeeder.setSmartCurrentLimit(20);
        m_pivotPIDController.setOutputRange(-1.0, 1.0);
        m_pivotPIDController.setFeedbackDevice(m_pivotEncoder);
        m_pivotPIDController.setP(ShooterConstants.kP);
        m_pivotPIDController.setI(ShooterConstants.kI);
        m_pivotPIDController.setD(ShooterConstants.kD);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ShooterConstants.kShooterForwardSoftLimit);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ShooterConstants.kShooterReverseSoftLimit);
        m_pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    private void initializeDistanceAngleLookup() {
        distanceAngleLookup = new ArrayList<>();
        distanceAngleLookup.add(new DistanceAnglePair(45.7, 0.85));
        distanceAngleLookup.add(new DistanceAnglePair(69.7, 0.82));
        distanceAngleLookup.add(new DistanceAnglePair(83.8, 0.805));
        distanceAngleLookup.add(new DistanceAnglePair(96.4, 0.793));
        distanceAngleLookup.add(new DistanceAnglePair(107.6, 0.789));
        distanceAngleLookup.add(new DistanceAnglePair(120.5, 0.783));
        distanceAngleLookup.add(new DistanceAnglePair(131.7, 0.776));
        distanceAngleLookup.add(new DistanceAnglePair(144.9, 0.772));
        distanceAngleLookup.add(new DistanceAnglePair(159.9, 0.768));
        distanceAngleLookup.add(new DistanceAnglePair(177.7, 0.764));
        distanceAngleLookup.add(new DistanceAnglePair(195.9, 0.76));
        distanceAngleLookup.add(new DistanceAnglePair(220.2, 0.757));
    }

    @Override
    public void periodic() {}

    public void pivotToSetPoint(double setPointRotations) {
        m_targetSetpointRotations = setPointRotations;

        m_pivotPIDController.setReference(setPointRotations, CANSparkMax.ControlType.kPosition);
    }

    public void pivotToSetPointAngle(double setPointAngle) {
        m_targetSetpointRotations = setPointAngle / 360.0;

        m_pivotPIDController.setReference(setPointAngle / 360.0, CANSparkMax.ControlType.kPosition);
    }

    public boolean isTargetInRestrictedRange(double targetRotations) {
        return targetRotations >= (1.0 / 360.0) && targetRotations <= (100.0 / 360.0);
    }

    public boolean isAtSetPoint() {
        double currentAngleRotations = m_pivotEncoder.getPosition();
        double toleranceRotations = 1.0 / 360.0;

        double error = Math.abs(currentAngleRotations - m_targetSetpointRotations);

        return error <= toleranceRotations;
    }

    public void shootMotors(boolean isPressed, double speed) {
        if (isPressed) {
            m_shooterMotorLeft.set(-speed);
            m_shooterMotorRight.set(speed * 0.75);
        } else {
            m_shooterMotorLeft.set(0);
            m_shooterMotorRight.set(0);
        }
    }

    public void invertMotor(boolean invert) {
        m_pivotMotor.setInverted(invert);
    }

    public void shooterDropNote(boolean isPressed, double speed) {
        double dropSpeed = -speed;

        if (isPressed) {
            m_shooterMotorLeft.set(-dropSpeed);
            m_shooterMotorRight.set(dropSpeed);
        } else {
            m_shooterMotorLeft.set(0);
            m_shooterMotorRight.set(0);
        }
    }

    public void moveFeeder(double speed) {
        m_shooterFeeder.set(speed);
    }

    public void shooterPickUpNote(boolean isPressed, double speed) {
        double pickUpSpeed = speed;

        if (isPressed) {
            m_shooterMotorLeft.set(-pickUpSpeed);
            m_shooterMotorRight.set(pickUpSpeed);
        } else {
            m_shooterMotorLeft.set(0);
            m_shooterMotorRight.set(0);
        }
    }

    public void setAngleBasedOnDistance(double distance) {
        if (distanceAngleLookup.isEmpty()) return;

        if (distance <= distanceAngleLookup.get(0).distance) {
            pivotToSetPoint(distanceAngleLookup.get(0).val);
            return;
        }
        if (distance >= distanceAngleLookup.get(distanceAngleLookup.size() - 1).distance) {
            pivotToSetPoint(distanceAngleLookup.get(distanceAngleLookup.size() - 1).val);
            return;
        }

        for (int i = 0; i < distanceAngleLookup.size() - 1; ++i) {
            if (distance >= distanceAngleLookup.get(i).distance && distance <= distanceAngleLookup.get(i + 1).distance) {
                double ratio = (distance - distanceAngleLookup.get(i).distance) /
                        (distanceAngleLookup.get(i + 1).distance - distanceAngleLookup.get(i).distance);
                double angle = distanceAngleLookup.get(i).val +
                        ratio * (distanceAngleLookup.get(i + 1).val - distanceAngleLookup.get(i).val);
                pivotToSetPoint(angle);
                return;
            }
        }
    }

    public void manualMove(double speed) {
        if (manualOverride) {
            double currentShooterPivotPosition = m_pivotEncoder.getPosition();

            double speedFactor = 0.01;
            double targetPositionRotations = currentShooterPivotPosition + (speed * speedFactor);

            if (!isTargetInRestrictedRange(targetPositionRotations)) {
                m_pivotPIDController.setReference(targetPositionRotations, CANSparkMax.ControlType.kPosition);
            }
        }
    }

    public boolean toggleManualOverride() {
        manualOverride = !manualOverride;
        return manualOverride;
    }

    public void stopMotors() {
        m_shooterMotorLeft.set(0);
        m_shooterMotorRight.set(0);
    }

    public double getShooterAngle() {
        return m_pivotEncoder.getPosition();
    }

    private static class DistanceAnglePair {
        final double distance;
        final double val;

        DistanceAnglePair(double distance, double val) {
            this.distance = distance;
            this.val = val;
        }
    }
}
