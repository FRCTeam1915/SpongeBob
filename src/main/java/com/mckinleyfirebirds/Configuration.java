package com.mckinleyfirebirds;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Configuration {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static double drivingFactor = Constants.kWheelDiameterMeters * Math.PI / Constants.kDrivingMotorReduction;
    static double turningFactor = 2 * Math.PI;
    static double drivingVelocityFeedForward = 1 / Constants.kDriveWheelFreeSpeedRps;

    static {
        drivingConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(50);

        drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);

        drivingConfig.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

        turningConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(20);

        turningConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0);
        turningConfig.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
    }
}
