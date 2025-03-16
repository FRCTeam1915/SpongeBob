package com.mckinleyfirebirds.commands;

import com.mckinleyfirebirds.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends Command {
    ElevatorSubsystem elevatorSubsystem;
    PIDController pidController;
    double level;

    public Elevator(ElevatorSubsystem elevatorSubsystem, double level) {
        this.elevatorSubsystem = elevatorSubsystem;
        pidController = new PIDController(3.0, 0.0, 0.05);
        this.level = level;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        double currentDistance = elevatorSubsystem.getMeasurement();
        double pidOutput = pidController.calculate(currentDistance, level);

        if (pidOutput > 1.0) pidOutput = 1.0; // Safety

        elevatorSubsystem.setSpeed(pidOutput);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.endMotors();
    }
}
