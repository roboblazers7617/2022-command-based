// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class ToggleMotors extends CommandBase {

  private final Drivetrain m_drivetrain;


  /** Creates a new ToggleMotors. */
  public ToggleMotors(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(0, .3, 0);
    Timer.delay(1);
    m_drivetrain.drive(0, 0, 0);
  Timer.delay(100);
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
