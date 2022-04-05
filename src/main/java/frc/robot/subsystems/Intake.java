// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;
import frc.robot.commands.Intake.ResetIntake;
import frc.robot.commands.Intake.ToggleIntakeRotation;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final PWMVictorSPX intakeMotor = new PWMVictorSPX(Constants.INTAKE_PORT);
  // private SparkMaxLimitSwitch fowardLimit;
  // private SparkMaxLimitSwitch backwordsLimit;
  //intake rotation motor is the motor that raises the intake off ground
  private final CANSparkMax intakeRotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_PORT,MotorType.kBrushless);

  //private boolean movingIntakeRotationMotor;
  //CLOSED IS CLOSED
  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;

  private boolean isIntakeZeroing = false;
  private boolean gravityLoweringIntake;
  private long timeUntilLowered;

  //private NetworkTableEntry speedDisplay = tab.add("Intake Motor Speed: ", 0).getEntry();
  private final NetworkTableEntry intakeRotationMotorPositionEntry, intakeMotorSpeedEntry, intakeRotationMotorSpeedEntry, intakeUpperLimitSwitchEntry, intakeLowerLimitSwitchEntry, intakeGravityDeployEntry;
  
  //private NetworkTableEntry intakeRotationSpeedDisplay = tab.add("Intake Rotation Motor Speed: ", 0).getEntry();
  private RelativeEncoder encoder = intakeRotationMotor.getEncoder();

  /** Creates a new Intake. */
  public Intake() {
    //intakeMotor.restoreFactoryDefaults();
    intakeRotationMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeRotationMotor.setInverted(true);

    intakeRotationMotor.setIdleMode(IdleMode.kBrake);
    //intakeMotor.setIdleMode(IdleMode.kCoast);


    gravityLoweringIntake = false;
    timeUntilLowered = 0;
    intakeRotationMotorPositionEntry = ShuffleboardInfo.getInstance().getIntakeRotationMotorPosition();
    intakeMotorSpeedEntry = ShuffleboardInfo.getInstance().getIntakeMotorSpeed();
    intakeRotationMotorSpeedEntry = ShuffleboardInfo.getInstance().getIntakeRotationMotorSpeed();
    intakeUpperLimitSwitchEntry = ShuffleboardInfo.getInstance().getIntakeUpperLimitSwitch();
    intakeLowerLimitSwitchEntry = ShuffleboardInfo.getInstance().getIntakeLowerLimitSwich();
    intakeGravityDeployEntry = ShuffleboardInfo.getInstance().getIntakeGravityDeploy();

    upperLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_UPPER_PORT);
    lowerLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_LOWER_PORT);
    encoder.setPosition(0);

    
  }
  /**sets the speed for the intake motor not the intake rotation motor */
  private void setSpeedIntake(double speed){//for intake motor
    if(isIntakeRotationMotorLowered()){
      intakeMotor.set(speed);
    }
  }
  public void startIntake(){
    setSpeedIntake(Constants.INTAKE_MOTOR_SPEED);
  }
  public void startIntakeReverse(){
    setSpeedIntake(-Constants.INTAKE_MOTOR_SPEED);
  }
  public void stopIntake(){
    setSpeedIntake(0.0);
  }
  /**gets the speed for the intake motor not the intake rotation motor */
  public double getSpeedIntake(){//for intake motor
    return intakeMotor.get();
  }

  public boolean isIntakeRotationMotorRaised(){
    //return !upperLimitSwitch.get() || (getEncoderPosition() >= (Constants.INTAKE_UPPER_ENCODER_VALUE - .7) && getSpeedIntakeRotation() == 0.0);
    if(isIntakeRasing()){
      if(!upperLimitSwitch.get() || encoder.getPosition() >= Constants.INTAKE_UPPER_ENCODER_VALUE -Constants.INTAKE_ROTATION_MOVEMENT_MAX_ERROR){
        return true;
      }
    }
    else{
      if(!upperLimitSwitch.get() || encoder.getPosition() >= Constants.INTAKE_UPPER_ENCODER_VALUE -Constants.INTAKE_ROTATION_CLASSFICATION_MAX_ERROR){
        return true;
      }
    }
    return false;
  }

  public boolean isIntakeRotationMotorLowered(){
   // return !lowerLimitSwitch.get() || (getEncoderPosition() <= (Constants.INTAKE_LOWER_ENCODER_VALUE + 1.0) && getSpeedIntakeRotation() == 0.0);
    if(isIntakeLowering()){
      if(!lowerLimitSwitch.get() || encoder.getPosition() <= Constants.INTAKE_LOWER_ENCODER_VALUE + Constants.INTAKE_ROTATION_MOVEMENT_MAX_ERROR){
        return true;
      }
    }
    else{
      if(!lowerLimitSwitch.get() || encoder.getPosition() <= Constants.INTAKE_LOWER_ENCODER_VALUE + Constants.INTAKE_ROTATION_CLASSFICATION_MAX_ERROR){
        return true;
      }
    }
    return false;
  }

  public double getSpeedIntakeRotation(){
    return intakeRotationMotor.get();
  }

  public void zeroIntake(){
    isIntakeZeroing = true;
    stopIntake();
    intakeRotationMotor.setIdleMode(IdleMode.kBrake);
    intakeRotationMotor.set(Constants.INTAKE_ZERO_OuT_MOTOR_SPEED);

  }

  
  public void cancelZeroIntake()
  {
    isIntakeZeroing = false;
    stopIntakeRotation();
  }

  public boolean isIntakeZeroingOut()
  {
    return isIntakeZeroing;
  }


  // public boolean isIntakeRotationMotorMoving(){
  //   return movingIntakeRotationMotor;
  // }

  public boolean isIntakeRasing(){
    return getSpeedIntakeRotation() > 0;
  }

  public boolean isIntakeLowering(){
    return getSpeedIntakeRotation() < 0;
  }

  public boolean isIntakeGravityLowering(){
    return gravityLoweringIntake;
  }

  public void stopIntakeRotation(){
    intakeRotationMotor.set(0.0);
  }


  /** will raise the intake up within the robot */
  public void raiseIntake(){
    if(!isIntakeRotationMotorRaised() || isIntakeLowering()){
      intakeRotationMotor.setIdleMode(IdleMode.kBrake);

      stopIntake();

      //Raising the instake is a positive direction
      intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED_UP_FAST);
    }
    

  }

  /** will lower the intake down to the ground, peramtor determines weather it will be a gravity deploy */
  public void lowerIntake(boolean gravity){
    if(isIntakeRotationMotorRaised() || isIntakeRasing()){
      gravityLoweringIntake = gravity;
      intakeRotationMotor.setIdleMode(IdleMode.kCoast);
      //Lowering the intake is a negative direction
      intakeRotationMotor.set(-Constants.INTAKE_ROTATION_MOTOR_SPEED_DOWN_FAST);
      if(gravity){
        
        gravityLoweringIntake = true;
        timeUntilLowered = System.currentTimeMillis() + Constants.INTAKE_GRAVITY_LOWER_TIME;
      }
    }

  }


  /**returns the encoder position for the intake rotation motor except for when testing */
  private double getEncoderPosition(){
    //return fakeEncoderPosition;
    return encoder.getPosition();
  }
  /**intake is raised */
  /*private boolean isUpperLimitSwitchTriped(){
    //return false;
    return upperLimitSwitch.get();
  }

  private boolean isLowerLimitSwitchTriped(){
    //return false;
    return lowerLimitSwitch.get();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeRotationMotorPositionEntry.setDouble(encoder.getPosition());
    intakeMotorSpeedEntry.setDouble(getSpeedIntake());
    intakeRotationMotorSpeedEntry.setDouble(encoder.getVelocity());
    intakeLowerLimitSwitchEntry.setBoolean(lowerLimitSwitch.get());
    intakeUpperLimitSwitchEntry.setBoolean(upperLimitSwitch.get());
    intakeGravityDeployEntry.setBoolean(gravityLoweringIntake);

    //intakeRotationSpeedDisplay.setDouble(intakeRotationMotor.get());//updates the display for the intake rotation motor speed
    //manage raise and lower intake
    //raise intake
    if(isIntakeZeroing)
    {
      if (!upperLimitSwitch.get())
      {
        stopIntakeRotation();
        encoder.setPosition(0);
        isIntakeZeroing = false;
      }
    }
    else if(isIntakeRasing()){
      if(encoder.getPosition() >= Constants.INTAKE_ENCODER_UPPER_SLOW_POSITION){
        intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED_UP_SLOW);
      }
      if(isIntakeRotationMotorRaised()){

        stopIntakeRotation();
      }
      
    }
    else if(isIntakeLowering()){
      if(encoder.getPosition() <= Constants.INTAKE_ENCODER_LOWER_SLOW_POSITION){
        intakeRotationMotor.set(-Constants.INTAKE_ROTATION_MOTOR_SPEED_DOWN_SLOW);
      }

      if(isIntakeRotationMotorLowered() && !gravityLoweringIntake){


        intakeRotationMotor.set(0.0);

        
      }
    }
    else if(gravityLoweringIntake){
      if(timeUntilLowered <= System.currentTimeMillis()){
        stopIntakeRotation();
        timeUntilLowered = 0;
      }
      if(isIntakeRotationMotorLowered()){
        gravityLoweringIntake = false;


      }
    }
/*
    if(isIntakeRotationMotorLowered() && encoder.getPosition() > Constants.INTAKE_LOWER_ENCODER_VALUE){
      intakeRotationMotor.setIdleMode(IdleMode.kCoast);
      encoder.setPosition(Constants.INTAKE_LOWER_ENCODER_VALUE);
    }
*/
    //used for testing purposes
    // if(raisingIntake){
    //   fakeEncoderPosition -= 0.05;
    // }
    // if(loweringIntake){
    //   fakeEncoderPosition += 0.05;
    // }
  }
}