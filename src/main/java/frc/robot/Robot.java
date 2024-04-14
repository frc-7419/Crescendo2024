// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
 
    Orchestra orchestra = new Orchestra();
    TalonFX[] motors = { new TalonFX(1, "Ryan Biggee"), new TalonFX(2, "Ryan Biggee"), new TalonFX(3, "Ryan Biggee"), new TalonFX(4, "Ryan Biggee"), new TalonFX(5, "Ryan Biggee"), new TalonFX(6, "Ryan Biggee"), new TalonFX(7, "Ryan Biggee"), new TalonFX(8, "Ryan Biggee")};
    private RobotContainer m_robotContainer;
    Thread m_visionThread;
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

        for (int i = 0; i < motors.length; ++i) {
            orchestra.addInstrument(motors[i]);
        }

        System.out.println(orchestra.loadMusic("flymetothemoon.chrp"));
        System.out.println("music");

        m_visionThread =

        new Thread(

            () -> {

              // Get the UsbCamera from CameraServer

              UsbCamera camera = CameraServer.startAutomaticCapture();

              // Set the resolution

              camera.setResolution(640, 480);


              // Get a CvSink. This will capture Mats from the camera

              CvSink cvSink = CameraServer.getVideo();

              // Setup a CvSource. This will send images back to the Dashboard

              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);


              // Mats are very memory expensive. Lets reuse this Mat.

              Mat mat = new Mat();


              // This cannot be 'true'. The program will never exit if it is. This

              // lets the robot stop this thread when restarting robot code or

              // deploying.

              while (!Thread.interrupted()) {

                // Tell the CvSink to grab a frame from the camera and put it

                // in the source mat.  If there is an error notify the output.

                if (cvSink.grabFrame(mat) == 0) {

                  // Send the output the error.

                  outputStream.notifyError(cvSink.getError());

                  // skip the rest of the current iteration

                  continue;

                }

                // Put a rectangle on the image

                Imgproc.rectangle(

                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

                // Give the output stream a new image to display

                outputStream.putFrame(mat);

              }

            });

    m_visionThread.setDaemon(true);

    m_visionThread.start();
        SignalLogger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
        // orchestra.play();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
