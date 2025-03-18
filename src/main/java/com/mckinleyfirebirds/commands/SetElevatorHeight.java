package com.mckinleyfirebirds.commands;

import com.mckinleyfirebirds.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorHeight extends Command {
    ElevatorSubsystem elevatorSubsystem;
    PIDController pidController;
    double height;

    // if side is true, it will go to the right side, otherwise it will go to the left side
    public SetElevatorHeight(ElevatorSubsystem elevatorSubsystem, double height) {
        this.elevatorSubsystem = elevatorSubsystem;
        pidController = new PIDController(3.5, 0.025, 0); // TODO: Need to fine tune this PID
        this.height = height;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        double pidOutput = pidController.calculate(elevatorSubsystem.getMeasurement(), height);
        System.out.println(elevatorSubsystem.getMeasurement() + " " + height);
        if (pidOutput > 0.75) pidOutput = 0.75; // Safety
//        elevatorSubsystem.setSpeed(pidOutput);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.endMotors();
    }
}
