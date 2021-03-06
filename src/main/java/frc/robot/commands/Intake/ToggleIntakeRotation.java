// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntakeRotation extends CommandBase {
  private final Intake intake;

  private boolean raising;
  /** Creates a new ToggleIntakeRotation. */
  public ToggleIntakeRotation(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //starts raised
    //rotating down is positive
    if(intake.isIntakeRotationMotorRaised()){
      intake.lowerIntake(false);
    }
    else{
      intake.raiseIntake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.isIntakeLowering() || intake.isIntakeRasing()){
      return false;
    }
    return true;
}
}
