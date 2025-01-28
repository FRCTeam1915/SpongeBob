// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import com.mckinleyfirebirds.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    DriveSubsystem driveSubsystem = new DriveSubsystem();
    XboxController controller = new XboxController(0);

    public RobotContainer() {
        configureBindings();
        driveSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> driveSubsystem.drive(
                                -MathUtil.applyDeadband(controller.getLeftY(), 0.05),
                                -MathUtil.applyDeadband(controller.getLeftX(), 0.05),
                                -MathUtil.applyDeadband(controller.getRightX(), 0.05),
                                true),
                        driveSubsystem));
    }

    private void configureBindings() {
        new JoystickButton(controller, PS4Controller.Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> driveSubsystem.setX(),
                        driveSubsystem
                ));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
