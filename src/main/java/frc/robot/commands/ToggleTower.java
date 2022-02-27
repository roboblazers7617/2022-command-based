// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class ToggleTower extends CommandBase {
  
  private Tower m_tower;
  
  /** Creates a new ToggleTower. */
  public ToggleTower(Tower tower) {
    m_tower = tower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_tower.getSpeedUpper())>0){
      m_tower.setSpeedUpper(0);
      m_tower.setSpeedLower(0);
      SmartDashboard.putNumber("Tower Speed", 0);
    }
    else{
      m_tower.setSpeedUpper(.5);
      m_tower.setSpeedLower(.5);
      SmartDashboard.putNumber("Tower Speed", .5);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
