// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax leftIntakeMotor;
    private final CANSparkMax rightIntakeMotor;
    private final CANSparkMax serializerBack;

    private final BeamBreakSubsystem beamBreakSubsystem;
    private double baselineCurrentDraw;
    private static final double CURRENT_THRESHOLD = 11.0;  //needs to be adjusted by testing

    public IntakeSubsystem(BeamBreakSubsystem beamBreakSubsystem) {
        leftIntakeMotor = new CANSparkMax(CanIds.leftIntakeMotor.id, MotorType.kBrushless);
        rightIntakeMotor = new CANSparkMax(CanIds.rightIntakeMotor.id, MotorType.kBrushless);

        this.beamBreakSubsystem = beamBreakSubsystem;
        serializerBack = new CANSparkMax(CanIds.serializerBack.id, MotorType.kBrushless);
        invertMotors();
        baselineCurrentDraw = serializerBack.getOutputCurrent();
    }

    public boolean frontBeamBreakIsTriggered() {
        return beamBreakSubsystem.frontBeamBreakIsTriggered();
    }

    public boolean backBeamBreakIsTriggered() {
        return !beamBreakSubsystem.backBeamBreakIsTriggered();
    }

    public void invertMotors() {
        serializerBack.setInverted(false);
        leftIntakeMotor.setInverted(true);
        rightIntakeMotor.setInverted(false);
    }

    //add voltage compensation and trapezoidal motion later
    public void setSpeed(double speed) {
        leftIntakeMotor.set(speed);
        rightIntakeMotor.set(speed);
    }

    public void setVoltage(double voltage) {
        leftIntakeMotor.setVoltage(voltage);
        rightIntakeMotor.setVoltage(voltage);
    }

    public void setSerializerVoltage(double voltage) {
        // serializerFront.setVoltage(voltage);
        serializerBack.setVoltage(voltage);
    }

    public void setSerializerSpeed(double speed) {
        // serializerFront.set(speed);
        serializerBack.set(speed);
    }

    public void brake() {
        leftIntakeMotor.setIdleMode(IdleMode.kBrake);
        rightIntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void brakeSerializer() {
        // serializerFront.setIdleMode(IdleMode.kBrake);
        serializerBack.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        leftIntakeMotor.setIdleMode(IdleMode.kCoast);
        rightIntakeMotor.setIdleMode(IdleMode.kCoast);
    }

    public void coastSerializer() {
        // serializerFront.setIdleMode(IdleMode.kCoast);
        serializerBack.setIdleMode(IdleMode.kCoast);
    }

    public double getVelocity() {
        return leftIntakeMotor.get();
    }
    public boolean noteDetectedByCurrent() {
        double currentDraw = leftIntakeMotor.getOutputCurrent();
        return Math.abs(currentDraw - baselineCurrentDraw) > CURRENT_THRESHOLD;
    }

    public void updateBaselineCurrentDraw() {
        baselineCurrentDraw = leftIntakeMotor.getOutputCurrent();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftIntakeSpeed", leftIntakeMotor.get());
        SmartDashboard.putNumber("RightIntakeSpeed", rightIntakeMotor.get());
        // SmartDashboard.putNumber("SerializerSpeed", serializerFront.get());
        SmartDashboard.putNumber("SerializerSpeed", serializerBack.get());
        SmartDashboard.putNumber("Current Draw", leftIntakeMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Has Note", frontBeamBreakIsTriggered());
    }
}
