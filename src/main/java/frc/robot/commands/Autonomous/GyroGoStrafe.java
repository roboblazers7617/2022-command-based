// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroGoStrafe extends CommandBase {
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
  private double speed;

  public GyroGoStrafe(Drivetrain drivetrain, String direction, double distance) {
        m_angleController = new PIDController(Constants.DRIVETRAIN_ROTATIONAL_KP,Constants.DRIVETRAIN_ROTATIONAL_KI,Constants.DRIVETRAIN_ROTAIONAL_KD);
        m_positionController = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
    // Use addRequirements() here to declare subsystem dependencies.
      m_drivetrain = drivetrain;
      finished = false;
      if(direction.equals("left") || direction.equals("Left")){
        m_direction = 1;
      }
      else if(direction.equals("right") || direction.equals("Right")){
        m_direction = -1;
      }
      else{
        m_direction = 0;
        finished = true;
      }
      m_distance = distance;
      addRequirements(drivetrain);
      m_gyroAdjustment = -m_drivetrain.getAngle();
      m_positionAdjustment = -m_drivetrain.getAverageEncoderPosition();
      m_angleController.setSetpoint(0+m_gyroAdjustment);
      m_positionController.setSetpoint(m_distance*m_direction+m_positionAdjustment);
      m_positionController.setTolerance(.05);
      m_angleController.enableContinuousInput(-180, 180);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angleAddjustment = m_angleController.calculate(m_drivetrain.getAngle());
    speed = (m_positionController.calculate(Units.inchesToMeters(m_drivetrain.getAverageEncoderPosition())));
    if(!finished){
    m_drivetrain.setSpeeds(speed-m_angleAddjustment, -speed+m_angleAddjustment,speed-m_angleAddjustment, -speed+m_angleAddjustment);
    if(m_positionController.atSetpoint()){
      finished = true;
    }
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
  return finished;
  }
}
