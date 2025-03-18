package com.mckinleyfirebirds.commands;

import com.mckinleyfirebirds.SpongeBob;
import com.mckinleyfirebirds.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDown extends Command {
    Timer timer;

    public ElevatorDown(ElevatorSubsystem elevatorSubsystem) {
        switch (elevatorSubsystem.getCurrentLevel()) {
            case LEVEL_ONE -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_FOUR);
            case LEVEL_TWO -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_ONE);
            case LEVEL_THREE -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_TWO);
            case LEVEL_FOUR -> elevatorSubsystem.setLevel(ElevatorLevel.LEVEL_THREE);
        }

        timer = new Timer();
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        SpongeBob.getInstance().setRumble(GenericHID.RumbleType.kLeftRumble, true);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() >= 1) {
            SpongeBob.getInstance().setRumble(GenericHID.RumbleType.kLeftRumble, false);
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SpongeBob.getInstance().setRumble(GenericHID.RumbleType.kLeftRumble, false);
    }
}