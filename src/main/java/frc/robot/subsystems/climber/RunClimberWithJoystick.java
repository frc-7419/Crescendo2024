// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RunClimberWithJoystick extends Command {
    private final Climber climber;
    private final CommandXboxController joystick;

    public RunClimberWithJoystick(Climber climber, CommandXboxController joystick) {
        this.climber = climber;
        this.joystick = joystick;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      climber.brake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ((Math.abs(joystick.getRightY()) > 0.05)) {
            climber.coast();
            climber.setVoltage(joystick.getRightY() * 5);
        }
        else{
          climber.setVoltage(0);
          climber.brake();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      climber.setVoltage(0);
       climber.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
