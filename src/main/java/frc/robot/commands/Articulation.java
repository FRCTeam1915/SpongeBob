package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Articulation extends Command {
    TalonSRX talon;

    CommandXboxController controller;
    boolean direction;
    boolean state = false;
    double speed;
    DutyCycleEncoder encoder;

    public Articulation(TalonSRX talon, CommandXboxController controller, boolean direction, DutyCycleEncoder encoder, double speed) {
        this.talon = talon;
        this.direction = direction;
        this.controller = controller;
        this.encoder = encoder;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        talon.set(ControlMode.PercentOutput, speed);
        state = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        talon.set(ControlMode.PercentOutput,0);
    }
}
