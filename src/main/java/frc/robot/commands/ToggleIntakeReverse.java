// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ToggleIntakeReverse extends CommandBase {
  private Intake intake;
  /** Creates a new Intake. */
  public ToggleIntakeReverse(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getSpeedIntake() == -Constants.INTAKE_MOTOR_SPEED){
      intake.setSpeedIntake(0.0);

    }
    else{
      intake.setSpeedIntake(-Constants.INTAKE_MOTOR_SPEED);
    }
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
