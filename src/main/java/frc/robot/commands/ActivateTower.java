// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;

public class ActivateTower extends CommandBase {
  private Tower tower;
  private boolean finished;
  /** Creates a new ToggleTower. */
  /*

  Both tower motors will spin until sensor is set to false(the laser is tripped)


  */
  public ActivateTower(Tower tower) {
    addRequirements(tower);
    this.tower = tower;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setSpeedLower(Constants.TOWER_SPEED);
    tower.setSpeedUpper(Constants.TOWER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(!tower.getLowerSensor()){
    //   tower.setSpeedLower(0);
      
    // }
    // if(!tower.getUpperSensor()){
    //   tower.setSpeedUpper(0);
    // }
    // if((tower.getSpeedLower() == 0.0) && (tower.getSpeedUpper() == 0.0)){
    //   finished = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.setSpeedUpper(0.0);
    tower.setSpeedLower(0.0);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return finished;
  }
}
