// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;

public class Climber extends SubsystemBase {

  
  private final CANSparkMax rightTopClimber = new CANSparkMax(Constants.RIGHT_TOP_CLIMBER_PORT, MotorType.kBrushless);
  private final RelativeEncoder rightTopClimberEncoder;


  private final CANSparkMax leftTopClimber = new CANSparkMax(Constants.LEFT_TOP_CLIMBER_PORT, MotorType.kBrushless);
  private final RelativeEncoder leftTopClimberEncoder;

  private final NetworkTableEntry climberTopLeftSpeedEntry;
  private final NetworkTableEntry climberTopRightSpeedEntry;
  private final NetworkTableEntry climberLimitSwitchEntry;
  private final NetworkTableEntry climberMaxEncoderValueEntry;


  /**the limit switch for the climber */
  private DigitalInput limitSwitch;

  private double maxEncoderValue;
  /** Creates a new Climber. */
  public Climber() {

    rightTopClimber.restoreFactoryDefaults();

    leftTopClimber.restoreFactoryDefaults();
    
    rightTopClimber.setIdleMode(IdleMode.kBrake);

    leftTopClimber.setIdleMode(IdleMode.kBrake);

    rightTopClimber.setInverted(true);

    leftTopClimberEncoder = leftTopClimber.getEncoder();
    rightTopClimberEncoder = rightTopClimber.getEncoder();



    climberTopRightSpeedEntry = ShuffleboardInfo.getInstance().getTopRightClimbEntry();
    climberTopLeftSpeedEntry = ShuffleboardInfo.getInstance().getTopLeftClimbEntry();
    climberLimitSwitchEntry = ShuffleboardInfo.getInstance().getClimberLimitSwitchEntry();
    climberMaxEncoderValueEntry = ShuffleboardInfo.getInstance().getClimberMaxEncodeEntry();

    limitSwitch = new DigitalInput(Constants.CLIMBER_LIMIT_PORT);

    maxEncoderValue = 100;
  }

  public void setSpeedTop(double leftSpeed, double rightSpeed){

   // leftClimber.set(speed);
    rightTopClimber.set(rightSpeed);
    leftTopClimber.set(leftSpeed);

  }

  

  

  public double getSpeedTopRight(){
    //  return leftClimber.get();
    return rightTopClimberEncoder.getVelocity();
  }

  public double getSpeedTopLeft(){
    //  return leftClimber.get();
    return leftTopClimberEncoder.getVelocity();
  }
  
  

  public boolean isClimberLowered(){
    boolean limitSwitchPosition = limitSwitch.get();
    return limitSwitchPosition;
  }

  public double getUpperEncoderLimit(){
    return maxEncoderValue;
  }

  public double getPositionRightMotor(){
    return rightTopClimberEncoder.getPosition();
  }

  @Override
  public void periodic() {
   // climberBottomSpeedEntry.setDouble(getSpeedBottom());
    climberTopRightSpeedEntry.setDouble(getSpeedTopRight());
    climberTopLeftSpeedEntry.setDouble(getSpeedTopLeft());
    climberLimitSwitchEntry.setBoolean(isClimberLowered());
    maxEncoderValue = climberMaxEncoderValueEntry.getDouble(100);
    //leftClimberDisplay.setDouble(getSpeedLeft());
    //rightClimberDisplay.setDouble(getSpeedRight());
    //setSpeed(climberToggle.getSelected());
    // This method will be called once per scheduler run

    
  }
}
