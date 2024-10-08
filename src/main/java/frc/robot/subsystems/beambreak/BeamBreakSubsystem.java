// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
    private final DigitalInput beamBreakFront;
    private final DigitalInput beamBreakBack;

    public BeamBreakSubsystem() {
        beamBreakFront = new DigitalInput(2);
        beamBreakBack = new DigitalInput(7);
    }

    public boolean frontBeamBreakIsTriggered() {
        return !beamBreakFront.get();
    }

    public boolean backBeamBreakIsTriggered() {
        return !beamBreakBack.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("frontBeamBreakTriggered", frontBeamBreakIsTriggered());
        SmartDashboard.putBoolean("backBeamBreakTriggered", backBeamBreakIsTriggered());
    }
}
