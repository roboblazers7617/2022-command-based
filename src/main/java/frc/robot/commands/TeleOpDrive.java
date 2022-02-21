// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class TeleOpDrive extends CommandBase {
  private final Supplier <Double> m_xSpeed;
  private final Supplier <Double> m_ySpeed;
  private final Supplier <Double> m_zRotation;
  private final Drivetrain m_drivetrain;

  /** Creates a new TeleOpDrive. */
  public TeleOpDrive(Drivetrain drivetrain, Supplier <Double> d, Supplier <Double> e, Supplier <Double> f) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_xSpeed = d;
    m_ySpeed = e;
    m_zRotation = f;
    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.drive(m_xSpeed.get(), m_ySpeed.get(), m_zRotation.get());

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
