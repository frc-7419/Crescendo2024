// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gyro;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.CanIds.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonSubsystem extends SubsystemBase {
  private Pigeon2 gyro;
  /** Creates a new PigeonSubsystem. */
  public PigeonSubsystem() {
    this.gyro = new Pigeon2(pigeon.id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyro yaw", getGyroYaw());
    SmartDashboard.putNumber("gyro pitch", getGyroPitch());
    SmartDashboard.putNumber("gyro roll", getGyroRoll());
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public double getGyroPitch() {
    return gyro.getPitch();
  }

  public double getGyroRoll() {
    return gyro.getRoll();
  }

}