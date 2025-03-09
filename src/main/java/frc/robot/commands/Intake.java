package frc.robot.commands;


import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    SparkFlex motor;
    double speed;

    public Intake(SparkFlex motor, double speed) {
        this.motor = motor;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        motor.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
    }
}
