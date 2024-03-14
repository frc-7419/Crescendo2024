package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class GoToShootPosition extends SequentialCommandGroup {
    Pose2d targetPose = new Pose2d(2.5,5.5, Rotation2d.fromDegrees(0));
    PathConstraints pathConstraints = new PathConstraints(
        3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0, 0);
    public GoToShootPosition(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            pathfindingCommand
        );
    }
}
