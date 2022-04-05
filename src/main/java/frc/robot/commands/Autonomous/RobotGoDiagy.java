// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import org.opencv.core.Mat;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RobotGoDiagy extends CommandBase {
  /** Creates a new RobotGoDiagy. */
  private Drivetrain m_drivetrain;
  private double m_distance;
  private int m_direction;
  private double endDistance;
  private boolean finished = false;
  public RobotGoDiagy(Drivetrain drivetrain, double distance, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_distance = distance;
    m_direction = direction;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endDistance = Units.inchesToMeters(m_distance) + m_drivetrain.getEncoderDistance();
    m_drivetrain.setBrakeMode("coast");
    if(m_direction == Constants.RIGHT){
      if(Math.signum(m_distance) == 1.0){
          m_drivetrain.setSpeeds(Constants.AUTO_SPEED, 0, 0, Constants.AUTO_SPEED);
      }
      else if(Math.signum(m_distance) == -1.0){
        m_drivetrain.setSpeeds(0, -Constants.AUTO_SPEED, -Constants.AUTO_SPEED, 0);
      }
      else{
        finished = true;
      }
    }

    if(m_direction == Constants.LEFT){
      if(Math.signum(m_distance) == 1.0){
        m_drivetrain.setSpeeds(0, Constants.AUTO_SPEED, Constants.AUTO_SPEED, 0);
    }
    else if(Math.signum(m_distance) == -1.0){
      m_drivetrain.setSpeeds(-Constants.AUTO_SPEED, 0, 0, -Constants.AUTO_SPEED);
    }
    else{
      finished = true;
    }
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.signum(m_distance) == 1.0){

      finished = m_drivetrain.getEncoderDistance() >= endDistance;
    }

    if(Math.signum(m_distance) == -1.0){
      finished = m_drivetrain.getEncoderDistance() <= endDistance;
    }
    return finished;
  }
}
