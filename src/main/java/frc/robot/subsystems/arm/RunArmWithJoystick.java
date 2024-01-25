// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RunArmWithJoystick extends Command {
  /** Creates a new RunArmWithJoystick. */
  private CommandXboxController joystick;
  private ArmSubsystem armSubsystem;
  private ArmFeedforward armFeedforward;
  private double maxPower = 0.1;
  public RunArmWithJoystick(ArmSubsystem armSubsystem, CommandXboxController joystick) {
    this.armSubsystem = armSubsystem;
    this.armFeedforward = new ArmFeedforward(0, 0.1, 0);
    this.joystick = joystick;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double feedForward = armFeedforward.calculate(, maxPower);
    double feedForward = 0.5/12;
    if (Math.abs(joystick.getLeftY())>0.05){
      armSubsystem.coast();
      double armPower = maxPower * joystick.getLeftY();
      armSubsystem.setPower(armPower+feedForward);
    } else {
      armSubsystem.setPower(feedForward);
      armSubsystem.brake();

    }
    // armSubsystem.setPower(joystick.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPower(0);
    armSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
