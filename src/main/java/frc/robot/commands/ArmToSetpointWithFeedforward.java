package frc.robot.commands;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.arm;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.ArmConstants;
// import frc.robot.constants.NodeConstants.NodeState;
// import frc.robot.subsystems.gripper.GripperSubsystem;
// import frc.robot.constants.PIDConstants;

// public class ArmToSetpointWithFeedforward extends Command {
//   /** Creates a new ArmToSetpointWithFeedforward. */
//   private IntakeSubsystem intakeSubsystem;

//   private double setpoint;
//   private TrapezoidProfile currentProfile;
//   private ArmFeedforward feedforward;
//   private PIDController armPIDController;

//   public ArmToSetpointWithFeedforward(IntakeSubsystem intakeSubsystem, NodeState intakesetpoint) {
//     this.intakeSubsystem = intakeSubsystem;
//     this.setpoint = intakesetpoint.armSetpoint;
//     this.feedforward = ArmConstants.armFeedforward;
//     this.armPIDController =
//         new PIDController(0.03, 0, 0);
//     addRequirements(intakeSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     intakeSubsystem.setGoal(setpoint);
//     double armPosition = intakeSubsystem.getAngle();
//     intakeSubsystem.setSetpoint(new TrapezoidProfile.State(armPosition, 0));

//     SmartDashboard.putNumber("Arm Goal Position", intakeSubsystem.getGoal().position);
//     SmartDashboard.putNumber("Arm Setpoint Position", intakeSubsystem.getSetpoint().position);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     currentProfile =
//         new TrapezoidProfile(
//             intakeSubsystem.getConstraints(), intakeSubsystem.getGoal(), intakeSubsystem.getSetpoint());

//     double currentPosition = intakeSubsystem.getAngle();

//     TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02);

//     // double feedForwardPower =
//     //     feedforward.calculate(nextSetpoint.position, nextSetpoint.velocity) / 12;

//     intakeSubsystem.setSetpoint(nextSetpoint);
    
//     armPIDController.setSetpoint(nextSetpoint.position);
    
//     double armPower = armPIDController.calculate(currentPosition);
//     double feedForwardPower = 
//     Math.copySign(ArmConstants.kg * Math.cos(Units.degreesToRadians(currentPosition)), armPower)/12;

//     intakeSubsystem.setPower(armPower);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intakeSubsystem.setPower(0);
//     intakeSubsystem.brake();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     double error = intakeSubsystem.getGoal().position - intakeSubsystem.getAngle();
//     boolean isAtSetpoint = Math.abs(error) <= 1;
//     return isAtSetpoint;
//   }
// }
