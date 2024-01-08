package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.NodeConstants;
import frc.robot.constants.NodeConstants.NodeState;


public class RobotContainer {
  private final XboxController driverJoystick = new XboxController(0);
  private final XboxController operatorJoystick = new XboxController(1);


  // // Autonomous
  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();


  // Path Planning Commands

  // TODO will use when testing path planning
  // private final MoveToMid moveToPortal = new MoveToMid(driveBaseSubsystem);

  public RobotContainer() {
    configureButtonBindings();
    configureAutoSelector();
  }

  private void configureButtonBindings() {
    
  }

  private void configureAutoSelector() {
    SmartDashboard.putData(autonomousChooser);
  }

  public Command getAutonomousCommand() {
    // ledSubsystem.rainbowLED(0);
     return autonomousChooser.getSelected();
  
  }

  public void setDefaultCommands() {

  }

  // public void zeroSensor(String optional, String allianceSide){}
  // public void zeroSensor(){}
}