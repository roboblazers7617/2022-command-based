// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class TeleOpDrive extends CommandBase {
  private final double m_xSpeed;
  private final double m_ySpeed;
  private final double m_zRotation;
  private final Drivetrain m_drivetrain;

  /** Creates a new TeleOpDrive. */
  public TeleOpDrive(Drivetrain drivetrain, double xSpeed, double ySpeed, double zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_zRotation = zRotation;
    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] speeds = {
      (m_xSpeed + m_ySpeed + m_zRotation),
      (m_xSpeed - m_ySpeed - m_zRotation),
      (m_xSpeed - m_ySpeed + m_zRotation),
      (m_xSpeed + m_ySpeed - m_zRotation)
    };

    double max = Math.abs(speeds[0]);
    for (int i = 0; i <speeds.length; i++){
      if (max < Math.abs(speeds[i])) max = Math.abs(speeds[1]);
    }

    if (max > 1) {
      for (int i = 0; i < speeds.length; i++) speeds [1] /= max;
    }

    m_drivetrain.setMotorSpeeds(speeds[0],speeds[1],speeds[2], speeds[3]);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
