// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends CommandBase {
  private final Shooter m_shooter;
  /** Creates a new SpinShooter. */
  public SpinShooter(Shooter shooter) {
    addRequirements(shooter);
    m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(Math.abs(m_shooter.getSpeed())>0){
      m_shooter.setSpeed(0);
      SmartDashboard.putNumber("Flywheel Speed", 0);
    }
    else{
      m_shooter.setSpeed(.33);
    }*/
    m_shooter.setSetPoint(Constants.SHOOTER_SPEED);
    m_shooter.startShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
