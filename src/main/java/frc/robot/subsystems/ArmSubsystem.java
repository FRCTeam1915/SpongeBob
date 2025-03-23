package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    DutyCycleEncoder armEncoder;
    TalonSRX armMotor;
    public ArmSubsystem(DutyCycleEncoder armEncoder, TalonSRX armMotor) {
        this.armEncoder = armEncoder;
        this.armMotor = armMotor;
    }
}
