// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.mckinleyfirebirds.commands.Articulate;
import com.mckinleyfirebirds.commands.Elevator;
import com.mckinleyfirebirds.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    TalonSRX talon = new TalonSRX(51);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        SpongeBob.getInstance().controller.a().onChange(new Elevator(elevatorSubsystem, 0.253));
        SpongeBob.getInstance().controller.b().onChange(new Elevator(elevatorSubsystem, 0.355));
        SpongeBob.getInstance().controller.x().onChange(new Elevator(elevatorSubsystem, 0.465));
        SpongeBob.getInstance().controller.y().onChange(new Elevator(elevatorSubsystem, 0.65));

        SpongeBob.getInstance().controller.leftBumper().whileTrue(new Articulate(talon, false));
        SpongeBob.getInstance().controller.rightBumper().whileTrue(new Articulate(talon, true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
