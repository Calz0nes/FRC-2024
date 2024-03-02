// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Placer extends SubsystemBase {
  /** Creates motors and names them. */
  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;

  /** Creates a new Placer. */
  public Placer() {
    backLeft = new CANSparkMax(Constants.launcherBackLeftCANID, MotorType.kBrushed);
    backRight = new CANSparkMax(Constants.launcherBackRightCANID, MotorType.kBrushed);
    frontLeft = new CANSparkMax(Constants.launcherFrontLeftCANID, MotorType.kBrushed);
    frontRight = new CANSparkMax(Constants.launcherFrontRightCANID, MotorType.kBrushed);
  }

  /** Creates the Action function to set speed of Placer motors. */
  public void Action(double PercentOutputFront, double PercentOutputBack) {
    frontLeft.set(-PercentOutputFront);
    frontRight.set(PercentOutputFront);
    backLeft.set(-PercentOutputBack);
    backRight.set(-PercentOutputBack);
  }

  /** Creates the Stop function to stop all Placer motors. */
  public void Stop() {
    frontLeft.set(0);
    frontRight.set(0);
    backLeft.set(0);
    backRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
