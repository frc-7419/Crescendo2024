// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkFlex shooterMotorTop;
  private CANSparkFlex shooterMotorBottom;

  private SparkPIDController topShooterPidController;
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806,0.015777);
  private SparkPIDController bottomShooterPidController;
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806,0.015777);

  private TalonFX shooterWrist;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  // setpoint needs to be set
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public ShooterSubsystem() {
    shooterMotorTop = new CANSparkFlex(CanIds.topShooter.id, MotorType.kBrushless);
    shooterMotorBottom = new CANSparkFlex(CanIds.bottomShooter.id, MotorType.kBrushless);

    topShooterPidController = shooterMotorTop.getPIDController();
    bottomShooterPidController = shooterMotorBottom.getPIDController();

    topShooterPidController.setP(0.001852);
    topShooterPidController.setI(0);
    topShooterPidController.setD(0);
    bottomShooterPidController.setP(0.001852);
    bottomShooterPidController.setI(0);
    bottomShooterPidController.setD(0);

    constraints = ShooterConstants.Constraints;
    shooterMotorTop.setSmartCurrentLimit(ShooterConstants.topShooterStallLimit, ShooterConstants.topShooterFreeLimit);
    shooterMotorBottom.setSmartCurrentLimit(ShooterConstants.bottomShooterStallLimit, ShooterConstants.bottomShooterFreeLimit);
    invertMotors();
  }

  public void setRPM(double topRPM, double bottomRPM) {
    shooterMotorTop.getPIDController().setFF(topFeedforward.calculate(topRPM));
    shooterMotorBottom.getPIDController().setFF(bottomFeedforward.calculate(bottomRPM));

    shooterMotorTop.getPIDController().setReference(topRPM, ControlType.kSmartVelocity);
    shooterMotorBottom.getPIDController().setReference(bottomRPM, ControlType.kSmartVelocity);
  }
  public void invertMotors() {
    shooterMotorBottom.setInverted(false);
    shooterMotorTop.setInverted(true);    
  }
  
  public void setTopSpeed(double speed){
    shooterMotorTop.set(speed);
  }

  public void setBottomSpeed(double speed){
    shooterMotorBottom.set(speed);
  }

  public void setBothSpeed(double speed){
    setTopSpeed(speed);
    setBottomSpeed(speed);
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
// 47