package com.mckinleyfirebirds.subsystems;

import com.mckinleyfirebirds.Constants;
import com.mckinleyfirebirds.Utils;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    MaxSwerveModule frontLeftModule = new MaxSwerveModule(
            Constants.FRONT_LEFT_DRIVING_MOTOR_ID,
            Constants.FRONT_LEFT_TURNING_MOTOR_ID,
            Constants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    );

    MaxSwerveModule frontRightModule = new MaxSwerveModule(
            Constants.FRONT_RIGHT_DRIVING_MOTOR_ID,
            Constants.FRONT_RIGHT_TURNING_MOTOR_ID,
            Constants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    MaxSwerveModule rearLeftModule = new MaxSwerveModule(
            Constants.REAR_LEFT_DRIVING_MOTOR_ID,
            Constants.REAR_LEFT_TURNING_MOTOR_ID,
            Constants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET
    );

    MaxSwerveModule rearRightModule = new MaxSwerveModule(
            Constants.REAR_RIGHT_DRIVING_MOTOR_ID,
            Constants.REAR_RIGHT_TURNING_MOTOR_ID,
            Constants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET
    );

    // Gyroscope sensor
    ADIS16470_IMU gyroscope = new ADIS16470_IMU();

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Constants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyroscope.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
            new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    rearLeftModule.getPosition(),
                    rearRightModule.getPosition()
            }
    );

    public DriveSubsystem() {
        //
    }

    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(gyroscope.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        rearLeftModule.getPosition(),
                        rearRightModule.getPosition()
                }
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(gyroscope.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        rearLeftModule.getPosition(),
                        rearRightModule.getPosition()
                },
                pose
        );
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * Constants.MAX_SPEED_IN_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeed * Constants.MAX_SPEED_IN_METERS_PER_SECOND;
        double rotationDelivered = rotationSpeed * Constants.MAX_ANGULAR_SPEED;

        SwerveModuleState[] swerveModuleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, Rotation2d.fromDegrees(gyroscope.getAngle(ADIS16470_IMU.IMUAxis.kZ)))
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED_IN_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        rearLeftModule.setDesiredState(swerveModuleStates[2]);
        rearRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void setX() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_IN_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        rearRightModule.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        rearLeftModule.resetEncoders();
        rearRightModule.resetEncoders();
    }

    public void zeroHeading() {
        gyroscope.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(gyroscope.getAngle(ADIS16470_IMU.IMUAxis.kZ)).getDegrees();
    }

    public double getTurnRate() {
        return gyroscope.getRate(ADIS16470_IMU.IMUAxis.kZ) * (Constants.GYROSCOPE_REVERSED ? 1.0 : -1.0);
    }

}
