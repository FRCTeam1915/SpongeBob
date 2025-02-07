package com.mckinleyfirebirds.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends Command {
    SparkMax motor1;
    SparkMax motor2;
    boolean direction;

    public Elevator(SparkMax motor1, SparkMax motor2, boolean direction) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.direction = direction;
    }
    @Override
    public void execute() {
        motor1.set(direction ? 0.3 : -0.3);
        motor2.set(direction ? -0.3 : 0.3);
    }

    @Override
    public void end(boolean interrupted) {
        motor1.stopMotor();
        motor2.stopMotor();
    }
}
