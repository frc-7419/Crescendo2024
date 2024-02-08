// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DeviceIDs.CanIds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import frc.robot.constants.RobotConstants.ShooterConstants;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class ShooterWrist extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX armMotor;
  private DutyCycleOut dutyCycleOut;
  private DutyCycleEncoder absoluteEncoder;
  Velocity<Angle> RotationsPerMinute = Rotations.per(Minute);

  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));

  public ShooterWrist() {
    armMotor = new TalonFX(CanIds.shooterWrist.id, "Ryan Biggee");
    dutyCycleOut = new DutyCycleOut(0);
    absoluteEncoder = new DutyCycleEncoder(0);
    absoluteEncoder.setPositionOffset(0.6056);
    // armMotor.setPosition(0);
  }
  private final SysIdRoutine shooterWristSysIdRoutine =
    new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        // Tell SysId how to plumb the driving voltage to the motors.
        (Measure<Voltage> volts) -> {
          armMotor.setVoltage(volts.in(Volts));
        },
        // Tell SysId how to record a frame of data for each motor on the mechanism being
        // characterized.
        log -> {
          // Record a frame for the left motors.  Since these share an encoder, we consider
          // the entire group to be one motor.
          log.motor("shooter-wrist")
            .voltage(
              appliedVoltage.mut_replace(
                armMotor.getDutyCycle().getValueAsDouble() * RobotController.getBatteryVoltage(), Volts))
            // .angularPosition(shooterDistance)
            .angularVelocity(RotationsPerMinute.of(getVelocity()))
           .angularPosition(Rotations.of(getPosition()));
          // Record a frame for the right motors.  Since these share an encoder, we consider
          // the entire group to be one motor.
        },
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("drive")
        this));

  public void setPower(double power){
    dutyCycleOut.Output = power;
    armMotor.setControl(dutyCycleOut);
  }
  public void setSetpoint(double setpoint){
    armMotor.setPosition(setpoint);
  }
  public void coast(){
    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void brake(){
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public void getAngle(){
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm position", getPosition());
  }

public double degreesToRotation(double degree) {
    return degree/360 * 70;
}

public double getPosition() {
  return armMotor.getPosition().getValueAsDouble() / 70;
}
public double getVelocity() {
  return armMotor.getVelocity().getValueAsDouble() / 70;
}
 public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return shooterWristSysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return shooterWristSysIdRoutine.dynamic(direction);
  }
}
