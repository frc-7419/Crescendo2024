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
  private final AddressableLEDWrapper led = new AddressableLEDWrapper(LEDConstants.PWMPORT, LEDConstants.BUFFERSIZE);
  private final BeamBreakSubsystem beamBreakSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterWrist shooterWrist;
  private final XboxController operatorController;
  private AddressableLEDWrapperPattern blue = new SolidColor(Color.kBlue);
	private AddressableLEDWrapperPattern red = new SolidColor(Color.kRed);
	private AddressableLEDWrapperPattern disabled = new DisabledPattern();
	private AddressableLEDWrapperPattern green = new SolidColor(Color.kGreen);
	private AddressableLEDWrapperPattern yellow = new SolidColor(Color.kLightYellow);
	private AddressableLEDWrapperPattern blinkingRed = new Blinking(Color.kRed, 0.25);
  private AddressableLEDWrapperPattern blinkingPurple = new Blinking(Color.kPurple, 0.25);
	private AddressableLEDWrapperPattern blinkingGreen = new Blinking(Color.kGreen, 0.25);
  private AddressableLEDWrapperPattern blinkingYellow = new Blinking(Color.kYellow, 0.25);
  private AddressableLEDWrapperPattern blinkingGold = new Blinking(Color.kGold, 1);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(BeamBreakSubsystem beamBreakSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ShooterWrist shooterWrist, XboxController operatorController) {
    this.beamBreakSubsystem = beamBreakSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterWrist = shooterWrist;
    this.operatorController = operatorController;
  }


  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
			led.setPattern(disabled);
      SmartDashboard.putString("LED Pattern", "disabled pattern");
    } else if (beamBreakSubsystem.frontBeamBreakIsTriggered()) {
      led.setPattern(green);
            SmartDashboard.putString("LED Pattern", "green");
    } else if (beamBreakSubsystem.backBeamBreakIsTriggered()) {
      led.setPattern(yellow);
            SmartDashboard.putString("LED Pattern", "yellow");

    } else if (Math.abs(intakeSubsystem.getVoltage()) > 1){
      led.setPattern(blinkingYellow);
      SmartDashboard.putString("LED Pattern", "blinkingYellow");
    } else if (operatorController.getRightBumper()){
      Double bs =  shooterSubsystem.getBottomPIDsetpoint();
      Double ts = shooterSubsystem.getTopPIDsetpoint();
      Double difference = (bs+ts) - (shooterSubsystem.getTopVelocity() + shooterSubsystem.getBottomVelocity());
      if(difference > 200){
        led.setPattern(blinkingPurple);
        SmartDashboard.putString("LED Pattern", "blinkingPurple");
      } else {
        led.setPattern(blinkingGreen);
        SmartDashboard.putString("LED Pattern", "blinkingGreen");
      } 
    } else if (operatorController.getBButton() || operatorController.getAButton() || operatorController.getYButton()){
      if((shooterWrist.getPIDsetpoint() - shooterWrist.getPosition() < 1)){
        led.setPattern(green);
        SmartDashboard.putString("LED Pattern", "green");
      } else {
        led.setPattern(blinkingRed);
        SmartDashboard.putString("LED Pattern", "blinkingRed");
      }
    } else {
      led.setPattern(blinkingGold);
      SmartDashboard.putString("LED Pattern", "blinkingGold");
    }
  }
}
