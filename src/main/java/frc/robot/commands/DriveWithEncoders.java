// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
 
public class DriveWithEncoders extends CommandBase {
  private double driveDistance;
  private double botSpeed;
  private double endDistance;
  private Drivetrain dt;
 
  //For simulation
  private double startTime;
  private double endTime;
 
  public DriveWithEncoders(Drivetrain drivetrain, double speed, double distanceInches) {
      addRequirements(drivetrain);
      dt = drivetrain;
      driveDistance =  distanceInches;
      botSpeed = speed;
  }
 
  // Called just before this Command runs the first time
  public void initialize() {
    // When this command is started, determine how far we want to drive
   
    endDistance = dt.getEncoderDistance() + driveDistance;
 
    // For simulation
    startTime = System.currentTimeMillis();
    endTime = startTime + 3000;
  }
 
  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    // Apply power to drive straight
    dt.drive(botSpeed, 0, 0);
 
  }
 
  // Called once after isFinished returns true
  public void end(boolean interrupted) {
      dt.drive(0,0,0);
  }
 
  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    // Stop once we've moved to or past the end distance
    //return (dt.getEncoderDistance() >= endDistance);
 
    //For simulation
    return (System.currentTimeMillis() >= endTime);
  }
 
}