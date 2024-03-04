// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class DisabledPattern implements AddressableLEDWrapperPattern{
	private Color[] colors = {Color.kNavy, Color.kGold};
	private int size;
	private int offset;
    
    public DisabledPattern(){
		super();
		
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int numberOfColors = colors.length;
		int effectiveIndex;
		int colorIndex;
		int bufferLength = buffer.getLength();
		for (int index = 0; index < bufferLength; index++){
			effectiveIndex = (index + offset) % bufferLength;
			colorIndex =( index /size )% numberOfColors;
			buffer.setLED(effectiveIndex, colors[colorIndex]);
		}

		offset =(offset+1) %bufferLength;
	}
	public boolean isAnimated(){
		return true;
    }
}
