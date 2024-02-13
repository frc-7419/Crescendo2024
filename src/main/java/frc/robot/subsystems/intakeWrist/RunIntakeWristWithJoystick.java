// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeWrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;

public class RunIntakeWristWithJoystick extends Command {
  /** Creates a new RunWristWithJoystick. */
  private IntakeWristSubsystem wrist;
  private CommandXboxController joystick;
  private double feedForward;
  public RunIntakeWristWithJoystick(IntakeWristSubsystem wrist, CommandXboxController joystick) {
    this.wrist = wrist;
    this.joystick = joystick;
    this.feedForward = - 0.07 * 0.2;
    addRequirements(wrist);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("joystick left y", joystick.getRightY());
    if(Math.abs(joystick.getRightY()) > 0.05){
      wrist.coast();
      double joystickWristPower = feedForward + joystick.getRightY() * RobotConstants.IntakeWristConstants.wristPower;
      wrist.setVoltage(joystickWristPower * 12);
      
    }
    else{
      wrist.setVoltage(feedForward * 12);
      wrist.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
    wrist.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
