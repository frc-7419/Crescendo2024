// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.NodeConstants.NodeState;
import frc.robot.constants.GripperConstants.GripperState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmToSetpoint;
import frc.robot.subsystems.arm.ArmToSetpointWithFeedforward;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorToSetpointWithFeedForward;
// import frc.robot.subsystems.elevator.ElevatorToSetpoint;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.RunGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartRetract extends SequentialCommandGroup {
  /** Creates a new ScorePiece. */

  /**
   * This command will score a game piece.
   *
   * @param elevatorSubsystem for controlling position of the elevator.
   * @param armSubsystem for controlling position of the arms.
   * @param gripperSubsystem for controlling orientation of the gripper.
   */
  public SmartRetract(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem,
      GripperSubsystem gripperSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ElevatorToSetpointWithFeedForward(elevatorSubsystem, NodeState.RESET),
        new ArmToSetpointWithFeedforward(armSubsystem, NodeState.RESET),
        new RunGripper(gripperSubsystem, GripperState.HOLD));
  }
}
