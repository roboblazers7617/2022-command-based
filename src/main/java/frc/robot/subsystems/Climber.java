// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;

public class Climber extends SubsystemBase {
 // private final PWMSparkMax leftClimber = new PWMSparkMax(Constants.LEFT_CLIMBER_PORT);
  
  private final CANSparkMax rightTopClimber = new CANSparkMax(Constants.RIGHT_TOP_CLIMBER_PORT, MotorType.kBrushless);
  private final RelativeEncoder rightTopClimberEncoder;
  private final CANSparkMax bottomClimber = new CANSparkMax(Constants.BOTTOM_CLIMBER_PORT, MotorType.kBrushless);
 private final RelativeEncoder bottomClimberEncoder;
  private final CANSparkMax leftTopClimber = new CANSparkMax(Constants.LEFT_TOP_CLIMBER_PORT, MotorType.kBrushless);
  private final RelativeEncoder leftTopClimberEncoder;
  private final NetworkTableEntry climberTopLeftSpeedEntry;
  private final NetworkTableEntry climberTopRightSpeedEntry;
  private final NetworkTableEntry climberBottomSpeedEntry;
  /** Creates a new Climber. */
  public Climber() {

    rightTopClimber.restoreFactoryDefaults();
     bottomClimber.restoreFactoryDefaults();
    leftTopClimber.restoreFactoryDefaults();
    
    rightTopClimber.setIdleMode(IdleMode.kBrake);
    bottomClimber.setIdleMode(IdleMode.kBrake);
    leftTopClimber.setIdleMode(IdleMode.kBrake);

    rightTopClimber.setInverted(true);

    leftTopClimberEncoder = leftTopClimber.getEncoder();
    rightTopClimberEncoder = rightTopClimber.getEncoder();
    bottomClimberEncoder = bottomClimber.getEncoder();

     climberBottomSpeedEntry = ShuffleboardInfo.getInstance().getBottomClimbEntry();
    climberTopRightSpeedEntry = ShuffleboardInfo.getInstance().getTopRightClimbEntry();
    climberTopLeftSpeedEntry = ShuffleboardInfo.getInstance().getTopRightClimbEntry();
  }

  public void setSpeedTop(double leftSpeed, double rightSpeed){

   // leftClimber.set(speed);
    rightTopClimber.set(rightSpeed);
    leftTopClimber.set(leftSpeed);

  }

  public void setSpeedBottom(double speed){

    bottomClimber.set(speed);
  }

  

  public double getSpeedTopRight(){
    //  return leftClimber.get();
    return rightTopClimberEncoder.getVelocity();
  }

  public double getSpeedTopLeft(){
    //  return leftClimber.get();
    return leftTopClimberEncoder.getVelocity();
  }
  
  public double getSpeedBottom(){
    return bottomClimberEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    climberBottomSpeedEntry.setDouble(getSpeedBottom());
    climberTopRightSpeedEntry.setDouble(getSpeedTopRight());
    climberTopLeftSpeedEntry.setDouble(getSpeedTopLeft());
    //leftClimberDisplay.setDouble(getSpeedLeft());
    //rightClimberDisplay.setDouble(getSpeedRight());
    //setSpeed(climberToggle.getSelected());
    // This method will be called once per scheduler run

    
  }
}
