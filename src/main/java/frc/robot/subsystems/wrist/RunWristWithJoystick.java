// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;

public class RunWristWithJoystick extends Command {
  /** Creates a new RunWristWithJoystick. */
  private Wrist wrist;
  private CommandXboxController joystick;
  public RunWristWithJoystick(Wrist wrist, CommandXboxController joystick) {
    this.wrist = wrist;
    this.joystick = joystick;
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
    if(Math.abs(joystick.getRightY()) > 0.05){
      double joystickArmPower = joystick.getLeftY() * RobotConstants.WristConstants.wristPower;
      wrist.setSpeed(joystick.getRightY());
    }
    else{
      wrist.setVoltage(0);
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
