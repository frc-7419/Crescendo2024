// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class DisabledPattern implements AddressableLEDWrapperPattern{
	public DisabledPattern(){
		super();
		
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
        for (int index = 0; index < buffer.getLength(); index++) {
            // Calculate the interpolation between gold and navy blue based on index
            int currentHue = interpolate(45, 240, index, buffer.getLength() - 1);
            buffer.setHSV(index, currentHue, 255, 128);
        }
    }

    // Linear interpolation function
    private int interpolate(int start, int end, int step, int steps) {
        return (int) (start + (end - start) * ((double) step / steps));
    }

    public boolean isAnimated() {
        return false;
    }
}
