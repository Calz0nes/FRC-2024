// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ArmPIDCMD;
import frc.robot.commands.AutoAllignCMD;
import frc.robot.commands.IntakeCMD;
import frc.robot.commands.PlacerCMD;
import frc.robot.commands.SpeakerCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Placer;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double TurtleModifier = 3;
  private final Pose2d AmpAllignment = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
  private final Pose2d SpeakerAllignment = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final Joystick equipmentController = new Joystick(1);

  // Define Subsystems here.
  final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Arm armSubsystem = new Arm();
  private final Intake intakeSubsystem = new Intake();
  private final Placer placerSubsystem = new Placer();

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

    // Brakes go brrr.
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // Points the modules.
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Reset the field-centric heading on left bumper press.
    joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Turtle Mode (slow).
    joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/TurtleModifier) // Drive forward with.
                                                                                           // negative Y (forward).
            .withVelocityY(-joystick.getLeftX() * MaxSpeed/TurtleModifier) // Drive left with negative X (left).
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate/TurtleModifier) // Drive counterclockwise with negative X (left).
    ));

    // Robot Alligns itself with Amp.
    joystick.leftBumper().whileTrue(new AutoAllignCMD(drivetrain, AmpAllignment));

    // Robot Alligns itself with Speaker.
    joystick.rightBumper().whileTrue(new AutoAllignCMD(drivetrain, SpeakerAllignment));

    /** Moves Arm up */
    new JoystickButton(equipmentController, 22).whileTrue(new ArmPIDCMD(armSubsystem, -23));

    /** Moves arm to speaker position */
    new JoystickButton(equipmentController, 23).whileTrue(new ArmPIDCMD(armSubsystem, -13));

    /** Moves Arm to loading Position */
    new JoystickButton(equipmentController, 24).whileTrue(new ArmPIDCMD(armSubsystem, -1));

    /** Intakes into Robot */
    new JoystickButton(equipmentController, 20).whileTrue(new IntakeCMD(intakeSubsystem, 0.7, 0.6));

    /** Intake spitz out of Robot */
    new JoystickButton(equipmentController, 19).whileTrue(new IntakeCMD(intakeSubsystem, -0.7, -0.6));

    /** Place Amp */
    new JoystickButton(equipmentController, 12).whileTrue(new PlacerCMD(placerSubsystem, -0.1, -0.1));

    /** Place Speaker */
    new JoystickButton(equipmentController, 0).whileTrue(new SpeakerCMD(placerSubsystem));

    /** Creeps Front Motors Backward*/
    new JoystickButton(equipmentController, 14).whileTrue(new PlacerCMD(placerSubsystem, -0.05, 0));

    /** Creeps Back Motors Backward*/
    new JoystickButton(equipmentController, 15).whileTrue(new PlacerCMD(placerSubsystem, 0, -0.05));

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
