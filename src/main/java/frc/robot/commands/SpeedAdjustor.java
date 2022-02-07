// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SpeedAdjustor extends CommandBase {
  private double m_speedModulator;
  private Drivetrain m_drivetrain;
  /** Creates a new SpeedAdjustor. */
  public SpeedAdjustor(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speedModulator = m_drivetrain.getSpeedModulator();
    if(m_speedModulator == Constants.HIGH_GEAR)
      m_drivetrain.setSpeedModulator(Constants.LOW_GEAR);
    else
      m_drivetrain.setSpeedModulator(Constants.HIGH_GEAR);
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
