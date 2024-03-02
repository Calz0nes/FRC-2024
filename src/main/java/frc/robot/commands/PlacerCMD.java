// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Placer;

public class PlacerCMD extends Command {

  private final Placer placerSubsystem;
  private final double speedFront, speedBack;

  /** Creates a new PlacerCMD. */
  public PlacerCMD(Placer placerSubsystem, double speedFront, double speedBack) {
    this.placerSubsystem = placerSubsystem;
    this.speedFront = speedFront;
    this.speedBack = speedBack;
    addRequirements(placerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    placerSubsystem.Action(speedFront, speedBack);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    placerSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
