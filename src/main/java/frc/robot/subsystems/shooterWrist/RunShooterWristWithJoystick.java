// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import com.team7419.util.MathUtil;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.IntakeWristConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class RunShooterWristWithJoystick extends Command {
    /**
     * Creates a new RunArmWithJoystick.
     */
    private final CommandXboxController joystick;
    private final ShooterWrist shooterWrist;
    // private ArmFeedforward armFeedforward;
    private final double maxPower = 1.0;
    private final double feedForward = (0.9 / 12) / 2.67 * 1;
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.02809 * 2.5, 0.01 * 1.5);

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
        if (Math.abs(joystick.getLeftY()) > 0.05) {
            shooterWrist.coast();
            double armJoystickPower = maxPower * -joystick.getLeftY() * 6;
            double feedForwardPower = armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocityInRadians());
            double powerWithFeedforward;
            
            if(armJoystickPower < 0){
                armJoystickPower = MathUtil.mappingClamp(armJoystickPower, -6, 0, -1, 0);
            }

            if(shooterWrist.getPosition() <= IntakeWristConstants.lowerLimit){
                powerWithFeedforward = Math.max(0, armJoystickPower) + feedForwardPower;
            }else if((shooterWrist.getPosition() >= IntakeWristConstants.upperLimit - 0.06) && (armJoystickPower > 0)){
                powerWithFeedforward = Math.max(0, (armJoystickPower - MathUtil.mappingClamp((shooterWrist.getPosition() - IntakeWristConstants.upperLimit), -0.06, 0.005, 0.0, armJoystickPower ))) + feedForwardPower;
            }else{
                powerWithFeedforward = armJoystickPower + feedForwardPower;
            } 
            
            shooterWrist.setPower(powerWithFeedforward);
            SmartDashboard.putNumber("powerWithFeedforward", powerWithFeedforward);
            //SmartDashboard.putNumber("armJoystickPower", armPower);
            SmartDashboard.putNumber("armFeedForward", powerWithFeedforward);

        } else {
            double feedForwardPower = armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocity());
            shooterWrist.setPower(feedForwardPower * 8);
            shooterWrist.brake();
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
