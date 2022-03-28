// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class LateralTraverse extends CommandBase {
  /** Creates a new LateralTraverse. */
  Drivetrain m_drivetrain;
  double m_xDistance;
  double m_yDistance;
  int goY;
  int goX;
  public LateralTraverse(Drivetrain drivetrain, double xDistance, double yDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    m_xDistance = xDistance+drivetrain.getRobotTranslation()[0];
    m_yDistance = yDistance+drivetrain.getRobotTranslation()[1];
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_xDistance > 0 ){
      goX = 1;
    }
    else if(m_xDistance < 0){
      goX = -1;
    }
    else{
      goX = 0;
    }

    if(m_yDistance > 0 ){
      goY = 1;
    }
    else if(m_yDistance < 0){
      goY = -1;
    }
    else{
      goY= 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_drivetrain.getRobotTranslation()[0] >= m_xDistance){
      goX = 0;
    }

    if(m_drivetrain.getRobotTranslation()[1] >= m_yDistance){
      goY = 0;
    }

    m_drivetrain.drive(Constants.AUTO_SPEED*goY, Constants.AUTO_SPEED*goX, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(m_drivetrain.getRobotTranslation()[0] >= m_xDistance && m_drivetrain.getRobotTranslation()[1] >= m_yDistance);
  }
}
