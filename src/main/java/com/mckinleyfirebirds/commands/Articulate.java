package com.mckinleyfirebirds.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;

public class Articulate extends Command {
    TalonSRX talon;
    boolean direction;

    public Articulate(TalonSRX talon, boolean direction) {
        this.talon = talon;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        talon.set(ControlMode.PercentOutput, direction ? 0.2 : -0.2);
    }

    @Override
    public void execute() {
//        System.out.println(talon.getSensorCollection().());
    }

    @Override
    public void end(boolean interrupted) {
        talon.set(ControlMode.PercentOutput, 0);
    }
}
