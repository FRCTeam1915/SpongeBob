package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Elevator extends Command {
    SparkMax motor1;
    SparkMax motor2;

    CommandXboxController controller;
    boolean direction;

    public Elevator(SparkMax motor1, SparkMax motor2, CommandXboxController controller, boolean direction) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.direction = direction;
        this.controller = controller;
    }
    @Override
    public void execute() {
        motor1.set(direction ? 0.5 : -0.5);
        motor2.set(direction ? -0.5 : 0.5);

        System.out.println(easeOut(controller.getLeftTriggerAxis()));
        System.out.println(easeOut(controller.getRightTriggerAxis()));

    }

    @Override
    public void end(boolean interrupted) {
        motor1.stopMotor();
        motor2.stopMotor();
    }

    double easeOut(double value) {
        return value * 0.5;
    }
}
