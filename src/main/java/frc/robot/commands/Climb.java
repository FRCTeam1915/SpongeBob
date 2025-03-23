package frc.robot.commands;


import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    SparkMax motor;
    double speed;

    public Climb(SparkMax motor, double speed) {
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
