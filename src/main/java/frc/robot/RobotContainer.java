// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double TurtleModify = 3;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

  // Define Subsystems here.
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Define Commands here.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Assign commands to buttons here.
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with.
                                                                                           // negative Y (forward).
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left).
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left).
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Turtle Mode (slow).
    joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/TurtleModify) // Drive forward with.
                                                                                           // negative Y (forward).
            .withVelocityY(-joystick.getLeftX() * MaxSpeed/TurtleModify) // Drive left with negative X (left).
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate/TurtleModify) // Drive counterclockwise with negative X (left).
    ));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /* Creates RobotContainer */
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
