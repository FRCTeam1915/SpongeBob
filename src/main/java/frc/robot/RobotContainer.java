// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ElevatorSubsystem;
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SparkMax motor3 = new SparkMax(35, SparkLowLevel.MotorType.kBrushless);
  TalonSRX talon1 = new TalonSRX(51);
  SparkFlex intakeMotor = new SparkFlex(33, SparkLowLevel.MotorType.kBrushless);
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();


  DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  static final         CommandXboxController driverXbox = new CommandXboxController(0);
  static final         CommandXboxController intakeXbox = new CommandXboxController(1);
//  final CommandXboxController operatorXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(Robot.getInstance().drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(Robot.getInstance().drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);
  SendableChooser<String> autoMode = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

//    NamedCommands.registerCommand("highElevator",new Elevator(motor1,motor2,intakeXbox));
//    NamedCommands.registerCommand("middleElevator",new Elevator(motor1,motor2,intakeXbox,2));
//    NamedCommands.registerCommand("lowElevator",new Elevator(motor1,motor2,intakeXbox,1));
//    NamedCommands.registerCommand("bottomElevator",new Elevator(motor1,motor2,intakeXbox,0));
//    NamedCommands.registerCommand("dropCoral", new Intake(intakeMotor,0.5));

    NamedCommands.registerCommand("Raise Elevator", new SetElevatorHeight(elevatorSubsystem, 0.665));
    DriverStation.silenceJoystickConnectionWarning(true);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
      intakeXbox.a().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.15));
      intakeXbox.b().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.23));
      intakeXbox.x().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.405));
      intakeXbox.y().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.665));

      intakeXbox.povDown().onTrue(new SetElevatorHeight(elevatorSubsystem, 0.055));



//    intakeXbox.b().toggleOnTrue(new Elevator2(motor1, motor2, intakeXbox));
//    intakeXbox.a().toggleOnTrue(new Elevator3(motor1, motor2, intakeXbox));
    intakeXbox.rightTrigger().whileTrue(new Intake(intakeMotor,0.2));
    intakeXbox.leftTrigger().whileTrue(new Intake(intakeMotor,-0.2));
    intakeXbox.rightBumper().whileTrue(new Articulation(talon1, intakeXbox, true, encoder,0.4));
    intakeXbox.leftBumper().whileTrue(new Articulation(talon1, intakeXbox, false, encoder,-0.4));
    intakeXbox.povUp().whileTrue(new Articulation(talon1, intakeXbox, true, encoder,-0.1));
    driverXbox.leftBumper().whileTrue(new Climb(motor3, 0.5));
    driverXbox.rightBumper().whileTrue(new Climb(motor3, -0.9));

    driverXbox.rightTrigger().whileTrue(new AimLeft(Robot.getInstance().drivebase));



    Command driveFieldOrientedDirectAngle      = Robot.getInstance().drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = Robot.getInstance().drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = Robot.getInstance().drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = Robot.getInstance().drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = Robot.getInstance().drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = Robot.getInstance().drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = Robot.getInstance().drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      Robot.getInstance().drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      Robot.getInstance().drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> Robot.getInstance().drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(Robot.getInstance().drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      Robot.getInstance().drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(Robot.getInstance().drivebase::lock, Robot.getInstance().drivebase).repeatedly());
      driverXbox.y().whileTrue(Robot.getInstance().drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(Robot.getInstance().drivebase::zeroGyro)));
      driverXbox.back().whileTrue(Robot.getInstance().drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(Robot.getInstance().drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(Robot.getInstance().drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          Robot.getInstance().drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(Robot.getInstance().drivebase::lock, Robot.getInstance().drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Robot.getInstance().drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    Robot.getInstance().drivebase.setMotorBrake(brake);
  }
}
