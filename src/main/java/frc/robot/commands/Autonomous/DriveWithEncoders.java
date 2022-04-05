// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands.Autonomous;
 
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
 
public class DriveWithEncoders extends CommandBase {
  private double driveDistanceMeters;
  private double botSpeed;
  private double endDistance;
  private Drivetrain dt;
  private boolean isDone;
  private double m_speedModulator = 1;
 
  public DriveWithEncoders(Drivetrain drivetrain, double distanceInches) {
      addRequirements(drivetrain);
      dt = drivetrain;
      driveDistanceMeters =  Units.inchesToMeters(distanceInches);
      botSpeed = Constants.AUTO_SPEED_FORWARD;
  }
 
  public DriveWithEncoders(Drivetrain drivetrain, double distanceInches, double speedModulator) {
    addRequirements(drivetrain);
    dt = drivetrain;
    m_speedModulator = speedModulator;
    driveDistanceMeters =  Units.inchesToMeters(distanceInches);
    botSpeed = Constants.AUTO_SPEED_FORWARD * speedModulator;
}

  // Called just before this Command runs the first time
  public void initialize() {
    // When this command is started, determine how far we want to drive
   
    endDistance = dt.getEncoderDistance() + driveDistanceMeters;
    
    
  }
 
  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    if (driveDistanceMeters < 0)
    {
      botSpeed = -Constants.AUTO_SPEED_FORWARD;
    }
    else if (driveDistanceMeters == 0)
      botSpeed = 0;

    else
      botSpeed = Constants.AUTO_SPEED_FORWARD;
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
    if(driveDistanceMeters >= 0){
      isDone = dt.getEncoderDistance() >= endDistance;
    }

    if(driveDistanceMeters < 0){
      isDone = dt.getEncoderDistance() <= endDistance;
    }

    return (isDone);
  }
 
}