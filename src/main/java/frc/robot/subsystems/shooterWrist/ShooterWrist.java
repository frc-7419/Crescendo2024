// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs.CanIds;

import static edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.Units.*;


public class ShooterWrist extends SubsystemBase {
    /**
     * Creates a new ArmSubsystem.
     */
    private final TalonFX armMotor;
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(5);
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final VoltageOut voltageOut;
    // Velocity<Angle> RotationsPerMinute = Rotations.per(Minute);
    private double PIDsetpoint;

    /*
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
             .angularPosition(getPosition().in(Rotations));
            // Record a frame for the right motors.  Since these share an encoder, we consider
            // the entire group to be one motor.
          },
          // Tell SysId to make generated commands require this subsystem, suffix test state in
          // WPILog with this subsystem's name ("drive")
          this));

          */
    public ShooterWrist() {
        armMotor = new TalonFX(CanIds.shooterWrist.id, "Ryan Biggee");
        voltageOut = new VoltageOut(12);
        armMotor.setInverted(true);
        armMotor.setPosition(0);
        encoder.setPositionOffset(0.587 - ArmConstants.armOffset.in(Rotations));

        // armMotor.enableVoltageCompensation(true);
    }

    public double getVelocity() {
        return armMotor.getVelocity().getValueAsDouble() / ArmConstants.armGearing;
    }

    public double getVelocityInRadians() {
        return getVelocity() * 2 * Math.PI;
    }

    public void setPower(double voltage) {
        voltageOut.Output = voltage;
        voltageOut.EnableFOC = true;
        armMotor.setControl(voltageOut);
  
    }

    public void setPowerUpward(double voltage) {
        setPower(voltage);
    }

    public void setMMConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V outpu
        // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        armMotor.getConfigurator().apply(talonFXConfigs);

    }

    public void applyRequest(double position) {
        final MotionMagicVoltage armRequest = new MotionMagicVoltage(position, true, 0.0, 0, false, false, false);
        armMotor.setControl(armRequest);
    }

    public void coast() {
        armMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brake() {
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getPosition() {
        return encoder.getAbsolutePosition() - encoder.getPositionOffset();
    }

    public double getPositionInRadians() {
        return getPosition() * 2 * Math.PI;
    }

    public double getPositionInDegrees() {
        return getPosition() * 360.0;
    }

    public double getPIDsetpoint() {
        return this.PIDsetpoint;
    }

    public void setPIDsetpoint(double setpoint) {
        this.PIDsetpoint = setpoint;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm position", getPosition());
        SmartDashboard.putNumber("Arm in Degrees", getPositionInDegrees());
    }
}
