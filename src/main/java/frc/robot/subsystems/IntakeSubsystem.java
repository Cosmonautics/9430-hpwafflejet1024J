package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intakePivotMotor;
    private final CANSparkMax m_intakeMotorLeft;
    private final SparkPIDController m_pidController;
    private final AbsoluteEncoder m_intakePivotAbsoluteEncoder;
    private double m_targetSetpointRotations;

    public IntakeSubsystem() {
        m_intakePivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotCanId, MotorType.kBrushless);
        m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeLeftCanId, MotorType.kBrushless);
        m_pidController = m_intakePivotMotor.getPIDController();
        m_intakePivotAbsoluteEncoder = m_intakePivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        configureMotors();
    }

    private void configureMotors() {
        m_intakePivotMotor.restoreFactoryDefaults();
        m_intakePivotMotor.setInverted(false);
        m_intakeMotorLeft.setSmartCurrentLimit(40);
        m_pidController.setOutputRange(-1.0, 1.0);
        m_pidController.setFeedbackDevice(m_intakePivotAbsoluteEncoder);
        m_pidController.setP(IntakeConstants.kP);
        m_pidController.setI(IntakeConstants.kI);
        m_pidController.setD(IntakeConstants.kD);
        m_intakePivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) IntakeConstants.kIntakeForwardSoftLimit);
        m_intakePivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) IntakeConstants.kIntakeReverseSoftLimit);
        m_intakePivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_intakePivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void intakeDrop(boolean isPressed, double speed) {
        double dropSpeed = 0.10;  // drop positive speed

        if (isPressed) {
            m_intakeMotorLeft.set(-dropSpeed);
        } else {
            m_intakeMotorLeft.set(0);
        }
    }

    public void pivotToAngle(double intakeAngleRotations, boolean movingDown) {
        if (movingDown) {
            m_pidController.setP(1.4);
        } else {
            m_pidController.setP(IntakeConstants.kP);
        }

        m_targetSetpointRotations = intakeAngleRotations;
        // Set the target position using PID controller
        m_pidController.setReference(intakeAngleRotations, CANSparkMax.ControlType.kPosition);
    }

    public void intakePickUp(boolean isPressed, double speed) {
        double pickUpSpeed = -speed;  // pick up is negative speed

        if (isPressed) {
            m_intakeMotorLeft.set(pickUpSpeed);
        } else {
            m_intakeMotorLeft.set(0);
        }
    }

    public void controlIntakeMotors(boolean isPressed, double speed) {
        if (isPressed) {
            m_intakeMotorLeft.set(-speed);
        } else {
            m_intakeMotorLeft.set(0);
        }
    }

    public boolean isAtSetPoint() {
        double currentAngleRotations = m_intakePivotAbsoluteEncoder.getPosition();
        double toleranceRotations = 1.0 / 360.0;  // Tolerance in rotations

        double error = Math.abs(currentAngleRotations - m_targetSetpointRotations);

        return error <= toleranceRotations;
    }

    public void stopMotors() {
        m_intakeMotorLeft.set(0);
    }
}
