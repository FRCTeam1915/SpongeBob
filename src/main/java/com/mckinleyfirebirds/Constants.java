package com.mckinleyfirebirds;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (13 * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Driving Parameters
    public static final double MAX_SPEED_IN_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    // Chassis Parameters
    public static final double TRACK_WIDTH = Units.inchesToMeters(17.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(33);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    // ===== Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;
    // =====

    // ===== SparkMax CAN IDs
    public static final int FRONT_LEFT_DRIVING_MOTOR_ID = 11;
    public static final int REAR_LEFT_DRIVING_MOTOR_ID = 13;
    public static final int FRONT_RIGHT_DRIVING_MOTOR_ID = 15;
    public static final int REAR_RIGHT_DRIVING_MOTOR_ID = 16;

    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 10; // L
    public static final int REAR_LEFT_TURNING_MOTOR_ID = 12;// L
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 14; // L
    public static final int REAR_RIGHT_TURNING_MOTOR_ID = 17;

    public static final boolean GYROSCOPE_REVERSED = false;
    // =====


    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final SparkBaseConfig.IdleMode kDrivingMotorIdleMode = SparkBaseConfig.IdleMode.kBrake;
    public static final SparkBaseConfig.IdleMode kTurningMotorIdleMode = SparkBaseConfig.IdleMode.kBrake;


    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
