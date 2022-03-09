// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;

public class ReverseTower extends CommandBase {
  private final Tower m_tower;
  /** Creates a new ReverseTower. */
  public ReverseTower(Tower tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tower = tower;
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tower.setSpeedLower(-Constants.TOWER_SPEED);
    m_tower.setSpeedUpper(-Constants.TOWER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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