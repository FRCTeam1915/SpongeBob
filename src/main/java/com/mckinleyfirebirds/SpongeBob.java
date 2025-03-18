// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SpongeBob extends TimedRobot {
    Command autonomousCommand;
    RobotContainer robotContainer;

    CommandXboxController controller = new CommandXboxController(0);

    DutyCycleEncoder encoder;

    public static SpongeBob instance;

    public SpongeBob() {
        instance = this;
    }

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

         encoder = new DutyCycleEncoder(10);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
//        SmartDashboard.putNumber("encoder", encoder.get());
//        System.out.println(encoder.get());
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopExit() {
//        new Elevator()
    }

    public void setRumble(GenericHID.RumbleType type, boolean on) {
        controller.setRumble(type, on ? 1 : 0);
    }

    public static SpongeBob getInstance() {
        return instance;
    }
}
