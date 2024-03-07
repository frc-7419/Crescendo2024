// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SolidColor implements AddressableLEDWrapperPattern{
	private Color m_color;

	public SolidColor(Color aColor){
		super();
		this.m_color = aColor;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		
		for (int index = 0; index < buffer.getLength(); index++){
			buffer.setLED(index, m_color);
		}
		
	}

}
