// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;

public class moveTowerIndividual extends CommandBase {
  private Tower m_tower;
  private Supplier<Double> m_speedUpper;
  private Supplier<Double> m_speedLower;
  /** Creates a new moveUpperTower. */
  public moveTowerIndividual(Tower tower, Supplier<Double> speedUpper, Supplier<Double> speedLower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tower=tower;
    m_speedUpper = speedUpper;
    m_speedLower = speedLower;
    addRequirements(m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if the joystick is pressed enough, the upper tower can be mannually controlled at fixed speeds
    if(Math.abs(m_speedUpper.get())>.2){
      if(m_speedUpper.get()>0)
        m_tower.setSpeedUpper(-Constants.UPPER_TOWER_SPEED);
      else 
        m_tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED);
    }
    else
      m_tower.setSpeedUpper(0);
    //if the joystick is pressed enough, the lower tower can be mannually controlled with variable speed
    if(Math.abs(m_speedLower.get())>.2){
      m_tower.setSpeedLower(m_speedLower.get()/Constants.NERF_LOWER_TOWER);
    }
    else{
      m_tower.setSpeedLower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if interupped, stop the tower
    m_tower.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //never end since it is effectivly a default command
    return false;
  }
}
