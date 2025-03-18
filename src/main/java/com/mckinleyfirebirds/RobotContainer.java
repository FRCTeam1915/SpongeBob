// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.mckinleyfirebirds.commands.Articulate;
import com.mckinleyfirebirds.commands.SetElevatorHeight;
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
//        SpongeBob.getInstance().controller.rightBumper().onTrue(new ElevatorUp(elevatorSubsystem));
//        SpongeBob.getInstance().controller.leftBumper().onTrue(new ElevatorDown(elevatorSubsystem));

        SpongeBob.getInstance().controller.a().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.055));
        SpongeBob.getInstance().controller.b().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.380));
        SpongeBob.getInstance().controller.x().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.480));
        SpongeBob.getInstance().controller.y().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.665));


        SpongeBob.getInstance().controller.leftBumper().whileTrue(new Articulate(talon, false));
        SpongeBob.getInstance().controller.rightBumper().whileTrue(new Articulate(talon, true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
