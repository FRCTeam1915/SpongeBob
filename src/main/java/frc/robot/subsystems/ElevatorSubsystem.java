package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax motor1;
    SparkMax motor2;
    LaserCan laser;

    public ElevatorSubsystem()  {
        motor1 = new SparkMax(60, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);

        laser = new LaserCan(25);

        try {
            laser.setRangingMode(LaserCanInterface.RangingMode.SHORT);
            laser.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS);
            laser.setRegionOfInterest(new LaserCanInterface.RegionOfInterest(2, 2, 8, 0));
        } catch (ConfigurationFailedException e) {
            System.err.println("HOW DID THE LASER FAIL!!! CHECK `ElevatorSystem` CLASS! THIS IS FETAL!");
        }
    }

    // Speed in percentage
    public void setSpeed(double speed) {
        motor1.set(speed);
        motor2.set(-speed);
    }

    public void endMotors() {
        motor1.stopMotor();
        motor2.stopMotor();
    }

    public double getMeasurement() {
        LaserCanInterface.Measurement measurement = laser.getMeasurement();
        // TODO: Check if we have invalid measurement
        SmartDashboard.putNumber("Elevator measurement", (double) measurement.distance_mm / 1000);
        return (double) measurement.distance_mm / 1000;
    }

//    public ElevatorLevel getCurrentLevel() {
//        return this.currentLevel;
//    }
//
//    public void setLevel(ElevatorLevel level) {
//        SmartDashboard.putString("Current Elevator Level", level.toString());
//        this.currentLevel = level;
//    }
}