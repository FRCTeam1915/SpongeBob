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
        this.direction = direction;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        System.out.println("THE ELEVATOR IS INITIALIZED");
        currentDistance = Utilities.distance;

        motor1.set(direction ? 0.8 : -0.8);
        motor2.set(direction ? -0.8 : 0.8);

        state = true;
    }

    @Override
    public void execute() {

            if ((Utilities.distance >= 750 && currentDistance < 750) || (Utilities.distance <= 250 && currentDistance > 250)) end(true);


//        if (currentDistance < 400) {
//            if (Utilities.distance >= 500) {
//                end(true);
//            }
//        }

//        if (Math.abs(Utilities.distance - currentDistance) > 100) {
//            end(true);
//        }

//        if (currentDistance > 400) {
//            if (Utilities.distance <= 500) {
//                end(true);
//            }
//        }

//        System.out.println("Old distance: " + currentDistance + " Current distance: " + Utilities.distance);
//        System.out.println("APPLIED VOTAGE" + motor1.getAppliedOutput() + " " + motor2.getAppliedOutput());
//        System.out.println("FACTOR: " + (Utilities.distance - currentDistance));
    }

    @Override
    public void end(boolean interrupted) {
        motor1.stopMotor();
        motor2.stopMotor();

        System.out.println("STOPPED!!!");
        currentDistance = Utilities.distance;
    }

    double easeOut(double value) {
        return value * 0.5;
    }
}
