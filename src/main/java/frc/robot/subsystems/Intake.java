// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates motors and names them. */
  private final CANSparkMax frontRoller;
  private final CANSparkMax backRoller;

  /** Creates a new Intake. */
  public Intake() {
    frontRoller = new CANSparkMax(Constants.intakeFrontCANID, MotorType.kBrushless);
    backRoller = new CANSparkMax(Constants.intakeBackCANID,MotorType.kBrushless);
  }

  /** Creates the Action function to set speed of Intake motors. */
  public void Action(double PercentOutputFront, double PercentOutputBack) {
    frontRoller.set(PercentOutputFront);
    backRoller.set(PercentOutputBack);
  }

  /** Creates the Stop function to stop all Intake motors. */
  public void Stop() {
    frontRoller.set(0);
    backRoller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
