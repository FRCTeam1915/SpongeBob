package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utilities;

public class Elevator extends Command {
    SparkMax motor1;
    SparkMax motor2;

    CommandXboxController controller;
    boolean direction;
    int currentDistance;

    boolean state = false;

    public Elevator(SparkMax motor1, SparkMax motor2, CommandXboxController controller, boolean direction) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.controller = controller;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        System.out.println("THE ELEVATOR IS INITIALIZED");
        currentDistance = Utilities.distance;

        motor1.set(direction ? 0.65 : -0.65);
        motor2.set(direction ? -0.65 : 0.65);



        state = true;
    }

    @Override
    public void execute() {
        if ((Utilities.distance >= 750 && currentDistance < 750) || (Utilities.distance <= 250 && currentDistance > 250)) {
            end(true);
        }


//        System.out.println("MOTOR 1 APPLIED OUTPUT -> " + motor1.getAppliedOutput() + " MOTOR 2 APPLIED OUTUT -> " + motor2.getAppliedOutput());

//        if ()
    }

    @Override
    public void end(boolean interrupted) {
        motor1.stopMotor();
        motor2.stopMotor();

        System.out.println("STOPPED!!!");
        currentDistance = Utilities.distance;
    }


}
