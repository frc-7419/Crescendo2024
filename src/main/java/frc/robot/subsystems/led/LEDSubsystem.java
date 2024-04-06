// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterWrist.ShooterWrist;

public class LEDSubsystem extends SubsystemBase {
    // BLUE AND GREEN ARE FLIPPED FOR SOME REASON
    private final AddressableLEDWrapper led = new AddressableLEDWrapper(LEDConstants.PWMPORT, LEDConstants.BUFFERSIZE);
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterWrist shooterWrist;
    private final XboxController operatorController;
    private final AddressableLEDWrapperPattern blue = new SolidColor(RGBtoRBG(Color.kBlue));
    private final AddressableLEDWrapperPattern red = new SolidColor(RGBtoRBG(Color.kRed));
    private final AddressableLEDWrapperPattern disabled = new DisabledPattern();
    private final AddressableLEDWrapperPattern green = new SolidColor(RGBtoRBG(Color.kGreen));
    private final AddressableLEDWrapperPattern yellow = new SolidColor(RGBtoRBG(Color.kLightYellow));
    private final AddressableLEDWrapperPattern blinkingRed = new Blinking(RGBtoRBG(Color.kRed), 0.25);
    private final AddressableLEDWrapperPattern blinkingPurple = new Blinking(RGBtoRBG(Color.kPurple), 0.25);
    private final AddressableLEDWrapperPattern blinkingGreen = new Blinking(RGBtoRBG(Color.kGreen), 0.25);
    private final AddressableLEDWrapperPattern blinkingYellow = new Blinking(RGBtoRBG(Color.kYellow), 0.25);
    private final AddressableLEDWrapperPattern blinkingGold = new Blinking(RGBtoRBG(Color.kGold), 1);
    private final AddressableLEDWrapperPattern off = new SolidColor(Color.kBlack);

    /**
     * Creates a new LEDSubsystem.
     */
    public LEDSubsystem(BeamBreakSubsystem beamBreakSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ShooterWrist shooterWrist, XboxController operatorController) {
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterWrist = shooterWrist;
        this.operatorController = operatorController;
    }

    private Color RGBtoRBG(Color hex) {
        String rgb = hex.toHexString();
        String rbg = rgb.substring(0, 3) + rgb.substring(5, 7) + rgb.substring(3, 5);
        return new Color(rbg);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            led.setPattern(disabled);
            // led.setPattern(off); // save power
            SmartDashboard.putString("LED Pattern", "disabled pattern");
        } else if (operatorController.getRightBumper()) {
            Double bs = shooterSubsystem.getBottomPIDsetpoint();
            Double ts = shooterSubsystem.getTopPIDsetpoint();
            Double difference = (bs + ts) - (shooterSubsystem.getTopVelocity() + shooterSubsystem.getBottomVelocity());
            if (difference > 200) {
                led.setPattern(blinkingPurple);
                SmartDashboard.putString("LED Pattern", "blinkingPurple");
            } else {
                led.setPattern(blinkingGreen);
                SmartDashboard.putString("LED Pattern", "blinkingGreen");
            }
        } else if (operatorController.getBButton() || operatorController.getAButton() || operatorController.getYButton()) {
            SmartDashboard.putNumber("shooter wrist difference", Math.abs(shooterWrist.getPIDsetpoint() - shooterWrist.getPosition()));
            if (Math.abs(shooterWrist.getPIDsetpoint() - shooterWrist.getPosition()) < 0.008) {
                led.setPattern(green);
                SmartDashboard.putString("LED Pattern", "green");
            } else {
                led.setPattern(blinkingRed);
                SmartDashboard.putString("LED Pattern", "blinkingRed");
            }
        } else if (beamBreakSubsystem.frontBeamBreakIsTriggered()) {
            led.setPattern(green);
            SmartDashboard.putString("LED Pattern", "green");
        } else if (beamBreakSubsystem.backBeamBreakIsTriggered()) {
            led.setPattern(yellow);
            SmartDashboard.putString("LED Pattern", "yellow");
        } else if (Math.abs(intakeSubsystem.getVelocity()) > 0.3) {
            led.setPattern(blinkingYellow);
            SmartDashboard.putString("LED Pattern", "blinkingYellow");
        } else {
            led.setPattern(blinkingGold);
            SmartDashboard.putString("LED Pattern", "blinkingGold");
        }
    }
}
