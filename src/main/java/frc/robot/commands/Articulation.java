package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utilities;

public class Articulation extends Command {
    TalonSRX talon;

    CommandXboxController controller;
    boolean direction;

    double currentAngle;

    boolean state = false;

    public Articulation(TalonSRX talon, CommandXboxController controller, boolean direction) {
        this.talon = talon;
        this.direction = direction;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        System.out.println("THE ELEVATOR IS INITIALIZED");
        currentAngle = Utilities.angle;
        talon.set(ControlMode.PercentOutput,direction ? 0.2 : -0.2);
        state = true;


    }

    @Override
    public void execute() {

//        System.out.println(Utilities.angle);

//        System.out.println(Utilities.distance);

//        if ((Utilities.distance >= 750 && currentAngle < 750) || (Utilities.distance <= 250 && currentAngle > 250)) end(true);


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
        talon.set(ControlMode.PercentOutput,0);

        System.out.println("STOPPED!!!");
        currentAngle = Utilities.angle;
    }

    double easeOut(double value) {
        return value * 0.5;
    }
}
