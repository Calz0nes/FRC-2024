// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;


public class AutoAllignCMD extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d desiredAllignment;
  private final PIDController translatePID, rotPID;
  private final boolean debug = true;

  ChassisSpeeds chassisSpeeds;
  Pose2d currPose;
  String ll = "limelight";


  /** Creates a new AutoAllignCMD. */
  public AutoAllignCMD(CommandSwerveDrivetrain drivetrain, Pose2d desiredAllignment) {
    this.drivetrain = drivetrain;
    this.desiredAllignment = desiredAllignment;
    this.translatePID = new PIDController(0.1, 0, 0);
    this.rotPID = new PIDController(0.1, 0, 0);
    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current positon.
    currPose = LimelightHelpers.getCameraPose3d_RobotSpace(ll).toPose2d();
    
    // Create new ChassisSpeeds object and calculate the speeds using our PID controllers.
    chassisSpeeds = new ChassisSpeeds(translatePID.calculate(currPose.getX(), desiredAllignment.getX()),
      translatePID.calculate(currPose.getY(), desiredAllignment.getY()),
      rotPID.calculate(currPose.getRotation().getRadians(), desiredAllignment.getRotation().getRadians()));

    // Litterly just sets the robot the the chassisSpeeds but this single line of code took me 30 minutes.
    drivetrain.applyRequest(() -> new SwerveRequest.ApplyChassisSpeeds().withSpeeds(chassisSpeeds));

    if (debug) {
    SmartDashboard.putNumber("OutPut VX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("OutPut VY", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("OutPut RotPS", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("LimeLight X", currPose.getX());
    SmartDashboard.putNumber("LimeLight Y", currPose.getY());
    SmartDashboard.putNumber("LimeLight Rot", currPose.getRotation().getRadians());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
