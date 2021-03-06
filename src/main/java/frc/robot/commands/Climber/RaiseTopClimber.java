// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RaiseTopClimber extends CommandBase {
  private final Climber climber;
  /** Creates a new RaiseTopClimber. */
  public RaiseTopClimber(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
 
   /* if (climber.getPositionRightMotor() <= climber.getUpperEncoderLimit() ){
        climber.setSpeedTop(Constants.LEFT_UPPER_CLIMBER_SPEED, Constants.RIGHT_UPPER_CLIMBER_SPEED);
    }    else{
      climber.setSpeedTop(0.0,0.0);
    }*/

    if (climber.getPositionRightMotor() <= climber.getUpperEncoderLimit())
    {
        climber.setSpeedRight(Constants.RIGHT_UPPER_CLIMBER_SPEED);
    } else {
      // stop the right motor
      climber.setSpeedRight(0.0);
    }
    
    if (climber.getPositionLeftMotor() <= climber.getUpperEncoderLimit()){
      climber.setSpeedLeft(Constants.LEFT_UPPER_CLIMBER_SPEED);
    } else {
      climber.setSpeedLeft(0.0);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeedTop(0.0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

