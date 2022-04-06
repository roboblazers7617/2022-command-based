// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroGoStraightSimple extends CommandBase {
  /** Creates a new GyroGoStraight. */
  private final Drivetrain m_drivetrain;
  private  double m_distance;
  private double m_angleAddjustment;
  private  int m_direction;
  private  double m_gyroSetpoint;
  private boolean finished = false;
  private double m_startingPosition;
  private double speed;
  private double endPosition;
  private double m_speedMultiplier = 1;
  private double kP = .00001;


  public GyroGoStraightSimple(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  
    m_drivetrain = drivetrain;
    m_distance = distance;
  }

  public GyroGoStraightSimple(Drivetrain drivetrain, double distance, double speedMultiplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  
    m_drivetrain = drivetrain;
      m_distance = distance;
      m_speedMultiplier = speedMultiplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_distance = Units.inchesToMeters(m_distance);
    m_gyroSetpoint = -m_drivetrain.getAngle();
    m_startingPosition = m_drivetrain.getEncoderDistance();
    endPosition = m_distance+m_startingPosition;

    if(m_distance > 0){
      m_direction = 1;
      finished = false;
    }
    else if(m_distance < 0){
      m_direction = -1;
      finished = false;
    }
    else{
      m_direction = 0;
      finished = true;
    }
    speed = m_direction*Constants.AUTO_SPEED_FORWARD*m_speedMultiplier;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!finished){
      m_angleAddjustment = kP*(m_gyroSetpoint+m_drivetrain.getAngle());
      m_drivetrain.setSpeeds(speed-m_angleAddjustment, speed+m_angleAddjustment, speed-m_angleAddjustment, speed+m_angleAddjustment);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_direction == 1){

      finished = m_drivetrain.getEncoderDistance() >= endPosition;
    }

    if(m_direction == -1){
      finished = m_drivetrain.getEncoderDistance() <= endPosition;
    }

    return (finished);
  }
}
