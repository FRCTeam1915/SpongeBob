package com.mckinleyfirebirds.commands;

import com.mckinleyfirebirds.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ChangeElevatorLevel extends Command {
    public ChangeElevatorLevel(ElevatorSubsystem elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
    }
}
