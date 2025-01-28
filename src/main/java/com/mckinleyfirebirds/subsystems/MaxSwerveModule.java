package com.mckinleyfirebirds.subsystems;

import com.mckinleyfirebirds.Configuration;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MaxSwerveModule {
    SparkMax drivingMotor;
    SparkMax turningMotor;

    RelativeEncoder drivingEncoder;
    AbsoluteEncoder turningEncoder;

    SparkClosedLoopController drivingClosedLoopController;
    SparkClosedLoopController turningClosedLoopController;

    double chassisAngularOffset;
    SwerveModuleState desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

    public MaxSwerveModule(int drivingMotorID, int turningMotorID, double chassisAngularOffset) {
        drivingMotor = new SparkMax(drivingMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        drivingClosedLoopController = drivingMotor.getClosedLoopController();
        turningClosedLoopController = turningMotor.getClosedLoopController();

        drivingMotor.configure(Configuration.drivingConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        turningMotor.configure(Configuration.turningConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        this.chassisAngularOffset = chassisAngularOffset;
        desiredModuleState.angle = new Rotation2d(turningEncoder.getPosition());

        drivingEncoder.setPosition(0.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredModuleState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredModuleState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredModuleState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
        turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), SparkBase.ControlType.kPosition);

        this.desiredModuleState = desiredModuleState;
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0.0);
    }
}
