// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class AdjustTower extends CommandBase {
  private Tower tower;
  private Shooter shooter;
  //checks if command should end
  private boolean finished; 
  /** Creates a new ToggleTower. */
  /*

  Both tower motors will spin until sensor is set to false(the laser is tripped)
  subsequent presses will turn the motors on and off, respectivily

  */
  public AdjustTower(Tower tower, Shooter shooter) {
    addRequirements(tower, shooter);
    this.tower = tower;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sensor to check if balls are in stored position
    if(shooter.getShooterSensor()){
      tower.setSpeedUpper(-.15);
    }

    if(!tower.upperSensor.get() && tower.getSpeedUpper()<0){
      tower.setSpeedUpper(0);
      tower.setSpeedLower(Constants.LOWER_TOWER_SPEED);
    }
    
      if(!tower.lowerSensor.get() && tower.getSpeedUpper()<0 && !shooter.getShooterSensor()){
        tower.setSpeedLower(0);
        finished = true; 
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tower.setSpeedUpper(0);
    tower.setSpeedLower(0);
  }
  
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends the command if the corresponding main is finished
  return finished;
  }
}
