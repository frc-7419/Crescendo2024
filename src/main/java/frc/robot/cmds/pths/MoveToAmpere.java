// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cmds.pths;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToAmpere extends SequentialCommandGroup {
  /** Creates a new MoveToAmpere. */
  private String pathAccordingToTeamColor = "";

  if (Robot.getAllianceColor().equals("Blue")) {

  }

  if (Robot.getAllianceColor().equals("Red")) {

  }

  public MoveToAmpere(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    List<PathPoint> pp = new ArrayList<PathPoint>();
    pp.add(new PathPoint(new Translation2d(1.9, 7.11),new RotationTarget(0.0, Rotation2d.fromDegrees(90))));
    PathPlannerTrajectory ppt = 
    addCommands();
  }
}
