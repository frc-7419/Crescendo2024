package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface AddressableLEDWrapperPattern {
    void setLEDs(AddressableLEDBuffer buffer);

    default boolean isAnimated() {
        return false;
    }
}