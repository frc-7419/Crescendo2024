// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkFlex shooterMotorTop;
  private CANSparkFlex shooterMotorBottom;
  private TalonFX shooterWrist;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  // setpoint needs to be set
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public ShooterSubsystem() {
    shooterMotorTop = new CANSparkFlex(CanIds.topShooter.id, MotorType.kBrushless);
    shooterMotorBottom = new CANSparkFlex(CanIds.bottomShooter.id, MotorType.kBrushless);
    constraints = ShooterConstants.Constraints;
    
    invertMotors();
  }

  public void invertMotors() {
    shooterMotorBottom.setInverted(false);
    shooterMotorTop.setInverted(false);    
  }
  
  public void setTopSpeed(double speed){
    shooterMotorTop.set(speed);
  }

  public void setBottomSpeed(double speed){
    shooterMotorBottom.set(speed);
  }

  public void setBothSpeed(double speed){
    setTopSpeed(speed);
    setBottomSpeed(-speed);
  }

  public void setTopVoltage(double voltage){
    shooterMotorTop.setVoltage(voltage);
  }

  public void setBottomVoltage(double voltage){
    shooterMotorBottom.setVoltage(voltage);
  }

  public void setBothVoltage(double voltage) {
    setTopVoltage(voltage);
    setBottomVoltage(-voltage);
  }
  
  public double getTopVelocity() {
    return shooterMotorTop.getEncoder().getVelocity();
  }

  public double getBottomVelocity() {
    return shooterMotorBottom.getEncoder().getVelocity();
  }

  public void brake() {
    shooterMotorTop.setIdleMode(IdleMode.kBrake);
    shooterMotorBottom.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    shooterMotorTop.setIdleMode(IdleMode.kCoast);
    shooterMotorBottom.setIdleMode(IdleMode.kCoast);
  }


  public void setGoal(double goalState) {
    goal = new TrapezoidProfile.State(goalState, 0);
  }

  public TrapezoidProfile.State getGoal() {
    return goal;
  }

  public void setWristSpeed(double speed) {
    shooterWrist.set(speed);
  }

  public void setWristVoltage(double voltage) {
    shooterWrist.setVoltage(voltage);
  }

  public void brakeWrist() {
    shooterWrist.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coastWrist() {
    shooterWrist.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setSetpoint(double setpointState) {
    setpoint = new TrapezoidProfile.State(setpointState, 0);
  }

  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Shooter Velocity", getTopVelocity());
    SmartDashboard.putNumber("Bottom Shooter Velocity", getBottomVelocity());
  }
  public Constraints getConstraints() {
      return constraints;
  }
}
