package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class GoToAmpPosition extends SequentialCommandGroup {
    private Pose2d targetPose;

    public GoToAmpPosition(CommandSwerveDrivetrain drivetrain) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        SmartDashboard.putBoolean("alliance present", alliance.isPresent());
        targetPose = new Pose2d(1.82, 7.76, Rotation2d.fromDegrees(-90));

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                targetPose = GeometryUtil.flipFieldPose(targetPose);
            }
        }

        PathConstraints pathConstraints = new PathConstraints(
                3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0, 0);

        addCommands(pathfindingCommand);
    }
}
