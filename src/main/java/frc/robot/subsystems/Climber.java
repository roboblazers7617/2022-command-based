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
  private final NetworkTableEntry climberLeftLimitSwitchEntry;
  private final NetworkTableEntry climberRightLimitSwitchEntry;
  private final NetworkTableEntry climberRightEncoderValueEntry;
  private final NetworkTableEntry climberLeftEncoderValueEntry;


  /**the limit switch for the climber */
  private DigitalInput limitSwitchLeft;
  private DigitalInput limitSwitchRight;

  private final double maxEncoderValue;
  /** Creates a new Climber. */
  public Climber() {

    rightTopClimber.restoreFactoryDefaults();

    leftTopClimber.restoreFactoryDefaults();
    
    rightTopClimber.setIdleMode(IdleMode.kBrake);

    leftTopClimber.setIdleMode(IdleMode.kBrake);

    rightTopClimber.setInverted(true);

    leftTopClimberEncoder = leftTopClimber.getEncoder();
    rightTopClimberEncoder = rightTopClimber.getEncoder();
    leftTopClimberEncoder.setPosition(0.0);
    rightTopClimberEncoder.setPosition(0.0);



    climberTopRightSpeedEntry = ShuffleboardInfo.getInstance().getTopRightClimbEntry();
    climberTopLeftSpeedEntry = ShuffleboardInfo.getInstance().getTopLeftClimbEntry();
    climberLeftLimitSwitchEntry = ShuffleboardInfo.getInstance().getClimberLimitSwitchEntry();
    climberRightLimitSwitchEntry = ShuffleboardInfo.getInstance().getClimberRightLimitSwitchEntry();
    climberRightEncoderValueEntry = ShuffleboardInfo.getInstance().getClimberRightEncoderValueEntry();
    climberLeftEncoderValueEntry = ShuffleboardInfo.getInstance().getClimberLeftEncoderValueEntry();

    limitSwitchLeft = new DigitalInput(Constants.CLIMBER_LIMIT_PORT); 
    limitSwitchRight = new DigitalInput(Constants.CLIMBER_LIMIT_PORT_RIGHT);

    maxEncoderValue = 132;
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
    boolean limitSwitchPositionLeft = !limitSwitchLeft.get();
    boolean limitSwithPositionRight = !limitSwitchRight.get();
    return (limitSwitchPositionLeft || limitSwithPositionRight);
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
    climberLeftLimitSwitchEntry.setBoolean(!limitSwitchLeft.get());
    climberRightLimitSwitchEntry.setBoolean(!limitSwitchRight.get());
    climberRightEncoderValueEntry.setDouble(rightTopClimberEncoder.getPosition());
    climberLeftEncoderValueEntry.setDouble(leftTopClimberEncoder.getPosition());
    //leftClimberDisplay.setDouble(getSpeedLeft());
    //rightClimberDisplay.setDouble(getSpeedRight());
    //setSpeed(climberToggle.getSelected());
    // This method will be called once per scheduler run

    
  }
}
