// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
    if(Math.abs(m_speedUpper.get())>.2){
      if(m_speedUpper.get()>0)
        m_tower.setSpeedUpper(-Constants.UPPER_TOWER_SPEED);
      else 
        m_tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED);
    }
    else
      m_tower.setSpeedUpper(0);

    if(Math.abs(m_speedLower.get())>.2){
      m_tower.setSpeedLower(m_speedLower.get()/-3);
    }
    else{
      m_tower.setSpeedLower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tower.setSpeedLower(0);
    m_tower.setSpeedUpper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
