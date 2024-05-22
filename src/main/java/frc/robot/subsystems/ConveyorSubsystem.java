package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
    private final CANSparkMax conveyorMotor;

    public ConveyorSubsystem() {
        conveyorMotor = new CANSparkMax(Constants.ConveyorConstants.kConveyorCanId, MotorType.kBrushless);
        conveyorMotor.setInverted(true);
        conveyorMotor.setSmartCurrentLimit(40);
        conveyorMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
    }

    public void forward() {
        conveyorMotor.set(1);
    }

    public void move(double speed) {
        conveyorMotor.set(speed);
    }

    public void reverse() {
        conveyorMotor.set(-1);
    }

    public void stop() {
        conveyorMotor.set(0);
    }
}
