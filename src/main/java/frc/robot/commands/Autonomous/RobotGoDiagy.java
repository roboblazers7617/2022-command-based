// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import javax.lang.model.util.ElementScanner6;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RobotGoDiagy extends CommandBase {
  /** Creates a new RobotGoDiagy. */
  private Drivetrain m_drivetrain;
  private int m_direction;
  private double endTime;
  private double driveTime;
  private boolean m_goForward = true;

  /// To get the robot to strafe diagonally, the x value needs to be higher than the y value. Changing the relationship of these
  // speeds will change the angle at which it strafes
  private double ySpeed = 0.3;
  private double xSpeed = 0.5;
  
  public RobotGoDiagy(Drivetrain drivetrain, double time, int directionLeftRight, boolean goForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    //m_distance = distance;
    m_direction = directionLeftRight;
    driveTime = time;
    m_goForward = goForward;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTime = System.currentTimeMillis() + driveTime*1000;

    // Because we are strafing, we will set the mode to coast to ensure the wheels spin freely. Need to set back to brake mode at
    // the end of the function
    m_drivetrain.setBrakeMode("coast");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_goForward == true)
    {
        if(m_direction == Constants.RIGHT){
          m_drivetrain.drive(ySpeed, xSpeed, 0);
        } else if(m_direction == Constants.LEFT){
            m_drivetrain.drive(ySpeed, -xSpeed, 0);
        } else {
          m_drivetrain.drive(0, 0, 0);
        }
    }else  { // go backwards diagonally
        if(m_direction == Constants.RIGHT){
          m_drivetrain.drive(-ySpeed, xSpeed, 0);
        } else if(m_direction == Constants.LEFT){
            m_drivetrain.drive(-ySpeed, -xSpeed, 0);
        } else {
            m_drivetrain.drive(0, 0, 0);
          }
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSpeeds(0, 0, 0, 0);
    m_drivetrain.setBrakeMode("brake");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() >= endTime);
   }
}
