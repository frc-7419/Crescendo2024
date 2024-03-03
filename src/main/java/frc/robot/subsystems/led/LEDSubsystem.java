// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.AddressableLEDWrapper;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLEDWrapper led = new AddressableLEDWrapper(LEDConstants.PWMPORT, LEDConstants.BUFFERSIZE);
  private final BeamBreakSubsystem beamBreakSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private AddressableLEDWrapperPattern blue = new SolidColor(Color.kBlue);
	private AddressableLEDWrapperPattern red = new SolidColor(Color.kRed);
	private AddressableLEDWrapperPattern disabled = new DisabledPattern();
	private AddressableLEDWrapperPattern green = new SolidColor(Color.kGreen);
	private AddressableLEDWrapperPattern yellow = new SolidColor(Color.kLightYellow);
	private AddressableLEDWrapperPattern blinkingRed = new Blinking(Color.kRed, 0.25);
  private AddressableLEDWrapperPattern blinkingPurple = new Blinking(Color.kPurple, 0.25);
	private AddressableLEDWrapperPattern blinkingGreen = new Blinking(Color.kGreen, 0.25);
  private AddressableLEDWrapperPattern blinkingYellow = new Blinking(Color.kYellow, 0.25);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(BeamBreakSubsystem beamBreakSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.beamBreakSubsystem = beamBreakSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }


  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
			led.setPattern(disabled);
    } else if (beamBreakSubsystem.frontBeamBreakIsTriggered()) {
      led.setPattern(green);
    } else if (beamBreakSubsystem.backBeamBreakIsTriggered()) {
      led.setPattern(yellow);
    } else if (intakeSubsystem.getVoltage() > 1){
      led.setPattern(blinkingYellow);
    }
    
    else {
      Double bs =  shooterSubsystem.getBottomPIDsetpoint();
      Double ts = shooterSubsystem.getTopPIDsetpoint();
      Double difference = (bs+ts) - (shooterSubsystem.getTopVelocity() + shooterSubsystem.getBottomVelocity());
      if(difference > 500){
        led.setPattern(blinkingPurple);
      } else {
        led.setPattern(blinkingGreen);
      }
    }
  }
}
