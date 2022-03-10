// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroGoStraight extends CommandBase {
  /** Creates a new GyroGoStraight. */
  private final Drivetrain m_drivetrain;
  private final double m_distance;
  private double m_angleAddjustment;
  private final int m_direction;
  private PIDController m_angleController;
  private PIDController m_positionController;
  private final double m_gyroAdjustment;
  private boolean finished;
  private final double m_positionAdjustment;

  public GyroGoStraight(Drivetrain drivetrain, String direction, double distance) {
        m_angleController = new PIDController(Constants.DRIVETRAIN_ROTATIONAL_KP,Constants.DRIVETRAIN_ROTATIONAL_KI,Constants.DRIVETRAIN_ROTAIONAL_KD);
        m_positionController = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
    // Use addRequirements() here to declare subsystem dependencies.
      m_drivetrain = drivetrain;
      finished = false;
      if(direction.equals("forward") || direction.equals("Forward")){
        m_direction = 1;
      }
      else if(direction.equals("backward") || direction.equals("Backward")){
        m_direction = -1;
      }
      else{
        m_direction = 0;
        finished = true;
      }
      m_distance = distance;
      addRequirements(drivetrain);
      m_gyroAdjustment = -m_drivetrain.getGyro();
      m_positionAdjustment = -m_drivetrain.getAverageEncoderPosition();
      m_angleController.setSetpoint(0+m_gyroAdjustment);
      m_positionController.setSetpoint(0+m_positionAdjustment);
      m_angleController.enableContinuousInput(-180, 180);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angleAddjustment = m_angleController.calculate(m_drivetrain.getGyro());
    if(!finished){
    m_drivetrain.setSpeeds((m_direction*Constants.AUTO_SPEED)-m_angleAddjustment, (m_direction*Constants.AUTO_SPEED)+m_angleAddjustment, (m_direction*Constants.AUTO_SPEED)-m_angleAddjustment, (m_direction*Constants.AUTO_SPEED)+m_angleAddjustment);
    if(Units.inchesToMeters(m_drivetrain.getAverageEncoderPosition()) == m_distance){
      finished = true;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return finished;
  }
}
