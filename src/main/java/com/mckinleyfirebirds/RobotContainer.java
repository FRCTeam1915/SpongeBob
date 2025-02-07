// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import com.mckinleyfirebirds.commands.Elevator;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    CommandXboxController controller = new CommandXboxController(0);

    SparkMax motor1 = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
    SparkMax motor2 = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        controller.leftTrigger().whileTrue(new Elevator(motor1, motor2, true));
        controller.rightTrigger().whileTrue(new Elevator(motor1, motor2, false));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
