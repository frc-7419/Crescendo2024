// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterMotor;
  private TrapezoidProfile.Constraints constraints;

  public Shooter() {
    shooterMotor = new CANSparkMax(CanIds.shooter.id, MotorType.kBrushless);
    constraints = 
      new TrapezoidProfile.Constraints(ShooterConstants.maxVelocity, ShooterConstants.maxAcceleration);
    zeroEncoder();
  }
  
  public void setSpeed(double speed){
    shooterMotor.set(speed);
  }

  public void setVoltage(double voltage){
    shooterMotor.setVoltage(voltage);
  }


  public double getVelocity() {
    return shooterMotor.getEncoder().getVelocity();
  }

  public void brake(){
    shooterMotor.setIdleMode(IdleMode.kBrake);
  }

  public void coast(){
    shooterMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
  }
  public Constraints getConstraints() {
      return constraints;
  }
}
