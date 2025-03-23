package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AimRight extends Command {

    SwerveSubsystem swerve;
    public AimRight(SwerveSubsystem swerveSubsystem) {
        this.swerve = swerveSubsystem;
    }
    @Override
    public void execute() {
        if (isValid()) {
//            if (getCurrentID() == 18) {
            swerve.drive(swerve.getTargetTraceSpeeds(getTranslationX() > 0.15 ? -3.5 : 3.5, 0.5, Rotation2d.fromDegrees(getYaw())));

//            swerve.drive(new Translation2d(getTranslationX() > 0.15 ? -1 : 1, 0), getYaw(), false);

            System.out.println("yaw: " + getYaw() + "TX: " + getTranslationX() + " TY: " + getTranslationY());
        }
    }

    public boolean isValid() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0) == 1;
    }

    public int getCurrentID() {
        return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);
    }

    public double getYaw() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[4];
    }

    public double getTranslationY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[1];
    }

    public double getTranslationX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[0];
    }
}
