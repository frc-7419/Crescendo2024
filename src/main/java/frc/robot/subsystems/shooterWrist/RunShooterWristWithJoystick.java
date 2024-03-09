// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RunShooterWristWithJoystick extends Command {
  /** Creates a new RunArmWithJoystick. */
  private CommandXboxController joystick;
  private ShooterWrist shooterWrist;
  // private ArmFeedforward armFeedforward;
  private double maxPower = 0.1;
  private double feedForward = (0.9/12)/2.67 * 1;
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.02809*2.5, 0.01*1.5);
  
  public RunShooterWristWithJoystick(ShooterWrist shooterWrist, CommandXboxController joystick) {
    this.shooterWrist = shooterWrist;
    // this.armFeedforward = new ArmFeedforward(0, 0.1, 0);
    this.joystick = joystick;
    addRequirements(shooterWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getLeftY()) > 0.05){
      shooterWrist.coast();
      double armJoystickPower = maxPower * -joystick.getLeftY() * 12;
      double feedForwardPower = armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocityInRadians());
      double powerWithFeedforward = armJoystickPower + feedForwardPower;
      SmartDashboard.putNumber("powerWithFeedforward", powerWithFeedforward);
      shooterWrist.setPower(powerWithFeedforward);
      //SmartDashboard.putNumber("armJoystickPower", armPower);
      SmartDashboard.putNumber("armFeedForward", powerWithFeedforward);

    } else {
      double feedForwardPower = armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocity());
      shooterWrist.setPower(feedForwardPower);
      SmartDashboard.putNumber("armFeedForward", feedForwardPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterWrist.setPower(0);
    shooterWrist.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
