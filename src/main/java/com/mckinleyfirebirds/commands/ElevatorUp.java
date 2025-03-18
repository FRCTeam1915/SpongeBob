package com.mckinleyfirebirds.commands;

import com.mckinleyfirebirds.SpongeBob;
import com.mckinleyfirebirds.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorUp extends Command {
    Timer timer;

    public ElevatorUp(ElevatorSubsystem elevatorSubsystem) {
        switch (elevatorSubsystem.getCurrentLevel()) {
            case LEVEL_ONE -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_TWO);
            case LEVEL_TWO -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_THREE);
            case LEVEL_THREE -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_FOUR);
            case LEVEL_FOUR -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_ONE);
        }

        timer = new Timer();

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        SpongeBob.getInstance().setRumble(GenericHID.RumbleType.kRightRumble, true);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Wait for the rumble duration
        if (timer.get() >= 1) {
            SpongeBob.getInstance().setRumble(GenericHID.RumbleType.kRightRumble, false);
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SpongeBob.getInstance().setRumble(GenericHID.RumbleType.kRightRumble, false);
    }
}
