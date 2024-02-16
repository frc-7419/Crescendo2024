package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.wrapper.VisionWrapper;

public class ShootingOnTheFly extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private ShooterSubsystem shooter;
    private VisionWrapper vision;

    public ShootingOnTheFly(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, VisionWrapper vision) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(drivetrain);
    }

}
