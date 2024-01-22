// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.intake;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Intake extends SubsystemBase {
//   /** Creates a new Intake. */
//   private CANSparkMax actuator;
//   private CANSparkMax run;
//   public Intake() {
//     actuator = new CANSparkMax(3, MotorType.kBrushless);
//     run = new CANSparkMax(2, MotorType.kBrushless);
//   }
//   public void setActuatorPower(double speed) {
//     actuator.set(speed);
//   }
//   public void setRunPower(double speed){
//     run.set(speed);
//   }
//   public void coastActuator() {
//     actuator.setIdleMode(IdleMode.kCoast);
//   }
//   public void coastRun() {
//     run.setIdleMode(IdleMode.kCoast);
//   }
//   public void brakeActuator() {
//     actuator.setIdleMode(IdleMode.kBrake);
//   }
//   public void brakeRun() {
//     actuator.setIdleMode(IdleMode.kBrake);
//   }
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
