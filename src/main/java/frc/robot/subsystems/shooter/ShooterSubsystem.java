// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    /**
     * Creates a new Shooter.
     */
    private final CANSparkFlex shooterMotorTop;
    private final CANSparkFlex shooterMotorBottom;
    private final RelativeEncoder topShooterEncoder;
    private final RelativeEncoder bottomShooterEncoder;

    private final SparkPIDController topShooterPidController;
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806, 0.015777);

    private final SparkPIDController bottomShooterPidController;
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806, 0.015777);

    private boolean isRunning;
    private double topPIDsetpoint;
    private double bottomPIDsetpoint;

    public ShooterSubsystem() {
        shooterMotorTop = new CANSparkFlex(CanIds.topShooter.id, MotorType.kBrushless);
        shooterMotorBottom = new CANSparkFlex(CanIds.bottomShooter.id, MotorType.kBrushless);

        topShooterEncoder = shooterMotorTop.getEncoder();
        bottomShooterEncoder = shooterMotorBottom.getEncoder();

        invertMotors();
        isRunning = true;

        shooterMotorTop.setSmartCurrentLimit(ShooterConstants.topShooterStallLimit, ShooterConstants.topShooterFreeLimit);
        topShooterPidController = shooterMotorTop.getPIDController();
        topShooterPidController.setP(0.00065 * 2);
        topShooterPidController.setI(0);
        topShooterPidController.setD(0);
        topShooterPidController.setIZone(0);
        topShooterPidController.setOutputRange(-1, 1);

        shooterMotorBottom.setSmartCurrentLimit(ShooterConstants.bottomShooterStallLimit, ShooterConstants.bottomShooterFreeLimit);
        bottomShooterPidController = shooterMotorBottom.getPIDController();
        bottomShooterPidController.setP(0.00065 * 2);
        bottomShooterPidController.setI(0);
        bottomShooterPidController.setD(0);
        bottomShooterPidController.setIZone(0);
        bottomShooterPidController.setOutputRange(-1, 1);
    }

    public void setRPM(double topRPM, double bottomRPM) {
        topShooterPidController.setFF(0.0003 / 2);
        bottomShooterPidController.setFF(0.0003 / 2);

        // System.out.println("changing rpm" + topRPM + bottomRPM);
        topShooterPidController.setReference(topRPM, ControlType.kVelocity);
        bottomShooterPidController.setReference(bottomRPM, ControlType.kVelocity);
    }

    public void invertMotors() {
        shooterMotorBottom.setInverted(true);
        shooterMotorTop.setInverted(false);
    }

    public void setTopSpeed(double speed) {
        shooterMotorTop.set(speed);
    }

    public void setBottomSpeed(double speed) {
        shooterMotorBottom.set(speed);
    }

    public void setBothSpeed(double speed) {
        setTopSpeed(speed);
        setBottomSpeed(speed);
    }

    public double getTopPIDsetpoint() {
        return this.topPIDsetpoint;
    }

    public void setTopPIDsetpoint(double setpoint) {
        this.topPIDsetpoint = setpoint;
    }

    public double getBottomPIDsetpoint() {
        return this.bottomPIDsetpoint;
    }

    public void setBottomPIDsetpoint(double setpoint) {
        this.bottomPIDsetpoint = setpoint;
    }

    public void setTopVoltage(double voltage) {
        shooterMotorTop.setVoltage(voltage);
    }

    public void setBottomVoltage(double voltage) {
        shooterMotorBottom.setVoltage(voltage);
    }

    public void setBothVoltage(double voltage) {
        setTopVoltage(voltage);
        setBottomVoltage(voltage);
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

    public boolean getToggle() {
        return isRunning;
    }

    public void invertToggle() {
        this.isRunning = !this.isRunning;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Top Shooter Velocity", getTopVelocity());
        SmartDashboard.putNumber("Bottom Shooter Velocity", getBottomVelocity());
        SmartDashboard.putBoolean("shooterToggle", isRunning);
    }
}
// 47