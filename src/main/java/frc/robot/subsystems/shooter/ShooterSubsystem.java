// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.RobotConstants.ShooterConstants;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkFlex shooterMotorTop;
  private CANSparkFlex shooterMotorBottom;
  private TalonFX shooterWrist;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  // setpoint needs to be set
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> shooterDistance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> shooterVelocity = mutable(MetersPerSecond.of(0));

  Velocity<Angle> RotationsPerMinute = Rotations.per(Minute);

  public ShooterSubsystem() {
    shooterMotorTop = new CANSparkFlex(CanIds.topShooter.id, MotorType.kBrushless);
    shooterMotorBottom = new CANSparkFlex(CanIds.bottomShooter.id, MotorType.kBrushless);
    constraints = ShooterConstants.Constraints;
    invertMotors();
  }

  private final SysIdRoutine shooterSysIdRoutine =
    new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        // Tell SysId how to plumb the driving voltage to the motors.
        (Measure<Voltage> volts) -> {
          shooterMotorBottom.setVoltage(volts.in(Volts));
          shooterMotorTop.setVoltage(volts.in(Volts));
        },
        // Tell SysId how to record a frame of data for each motor on the mechanism being
        // characterized.
        log -> {
          // Record a frame for the left motors.  Since these share an encoder, we consider
          // the entire group to be one motor.
          log.motor("shooter-bottom")
            .voltage(
              appliedVoltage.mut_replace(
                shooterMotorTop.get() * RobotController.getBatteryVoltage(), Volts))
            // .angularPosition(shooterDistance)
            .angularVelocity(RotationsPerMinute.of(getBottomVelocity()))
           .angularPosition(Rotations.of(getBottomRotations()));
          // Record a frame for the right motors.  Since these share an encoder, we consider
          // the entire group to be one motor.
          log.motor("shooter-top")
            .voltage(
              appliedVoltage.mut_replace(
                shooterMotorTop.get() * RobotController.getBatteryVoltage(), Volts))
             .angularVelocity(RotationsPerMinute.of(getTopVelocity()))
           .angularPosition(Rotations.of(getTopRotations()));
        },
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("drive")
        this));

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

  public double getTopVoltage(){
    return shooterMotorTop.getBusVoltage();
  }
  public double getBottomVoltage(){
    return shooterMotorBottom.getBusVoltage();
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
    SmartDashboard.putNumber("Top Shooter RPM", getTopVelocity());
    SmartDashboard.putNumber("Bottom Shooter RPM", getBottomVelocity());
    // SmartDashboard.putNumber("Shooter Wrist Position", shooterWrist.getS);
  }
  public Constraints getConstraints() {
      return constraints;
  }

  public double getTopVelocityMetersPerSecond() {
    return (getTopVelocity()*Math.PI*ShooterConstants.wheelDiameterIn*0.0254)/60;
  }

  public double getBottomVelocityMetersPerSecond() {
    return (getBottomVelocity()*Math.PI*ShooterConstants.wheelDiameterIn*0.0254)/60;
  }

  public double getTopPositionMeters() {
    return (shooterMotorTop.getEncoder().getPosition()*Math.PI*ShooterConstants.wheelDiameterIn*0.0254);
  }

  public double getBottomPositionMeters() {
    return (shooterMotorBottom.getEncoder().getPosition()*Math.PI*ShooterConstants.wheelDiameterIn*0.0254);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return shooterSysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return shooterSysIdRoutine.dynamic(direction);
  }
  public double getTopRotations() {
    return shooterMotorTop.getEncoder().getPosition();
  }
  public double getBottomRotations() {
    return shooterMotorBottom.getEncoder().getPosition();
  }
}
