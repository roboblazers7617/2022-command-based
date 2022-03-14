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
import frc.robot.commands.ResetIntake;
import frc.robot.commands.ToggleIntakeRotation;
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
  private boolean intakeRotationMotorRaised;
  //private boolean movingIntakeRotationMotor;
  //CLOSED IS CLOSED
  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;
  private boolean raisingIntake;
  private boolean loweringIntake;
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  //private NetworkTableEntry speedDisplay = tab.add("Intake Motor Speed: ", 0).getEntry();
  private final NetworkTableEntry intakeRotationMotorPositionEntry, intakeMotorSpeedEntry, intakeRotationMotorSpeedEntry, intakeUpperLimitSwitchEntry, intakeLowerLimitSwitchEntry;
  
  private double fakeEncoderPosition;
  
  //private NetworkTableEntry intakeRotationSpeedDisplay = tab.add("Intake Rotation Motor Speed: ", 0).getEntry();
  private RelativeEncoder encoder = intakeRotationMotor.getEncoder();

  /** Creates a new Intake. */
  public Intake() {
    //intakeMotor.restoreFactoryDefaults();
    intakeRotationMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeRotationMotor.setInverted(true);
    intakeRotationMotorRaised = true;
    intakeRotationMotor.setIdleMode(IdleMode.kCoast);
    //intakeMotor.setIdleMode(IdleMode.kCoast);
    raisingIntake = false;
    loweringIntake = false;
    intakeRotationMotorPositionEntry = ShuffleboardInfo.getInstance().getIntakeRotationMotorPosition();
    intakeMotorSpeedEntry = ShuffleboardInfo.getInstance().getIntakeMotorSpeed();
    intakeRotationMotorSpeedEntry = ShuffleboardInfo.getInstance().getIntakeRotationMotorSpeed();
    intakeUpperLimitSwitchEntry = ShuffleboardInfo.getInstance().getIntakeUpperLimitSwitch();
    intakeLowerLimitSwitchEntry = ShuffleboardInfo.getInstance().getIntakeLowerLimitSwich();

    upperLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_UPPER_PORT);
    lowerLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_LOWER_PORT);
    encoder.setPosition(0);
    fakeEncoderPosition = 0.0;

    
  }
  /**sets the speed for the intake motor not the intake rotation motor */
  public void setSpeedIntake(double speed){//for intake motor
    if(!intakeRotationMotorRaised){
      intakeMotor.set(speed);
    }
  }
  /**gets the speed for the intake motor not the intake rotation motor */
  public double getSpeedIntake(){//for intake motor
    return intakeMotor.get();
  }

  public boolean getIntakeRotationMotorRaised(){
    return intakeRotationMotorRaised;
  }

  // public boolean isIntakeRotationMotorMoving(){
  //   return movingIntakeRotationMotor;
  // }

  public boolean isIntakeRasing(){
    // if(getUpperLimitSwitch() || getEncoderPosition()() < Constants.INTAKE_UPPER_ENCODER_VALUE){
    //   return false;
    // }
    // return true;
    return raisingIntake;
  }

  public boolean isIntakeLowering(){
    // if(getLowerLimitSwitch() || getEncoderPosition()() > Constants.INTAKE_LOWER_ENCODER_VALUE){
    //   return false;
    // }
    // return true;
    return loweringIntake;
  }

  public void stopIntakeRotation(){
    intakeRotationMotor.set(0.0);
  }


  

  /** will raise the intake up within the robot */
  public void raiseIntake(){
    //SparkMaxLimitSwitch limit = intakeRotationMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED);

    // //used this for help https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Soft%20Limits/src/main/java/frc/robot/Robot.java 
    // intakeRotationMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    // intakeRotationMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.INTAKE_ROTATION_MOTOR_DISTANCE);
    
    // intakeRotationMotorRaised = true;
    raisingIntake = true;
    setSpeedIntake(0.0);

    //Raising the instake is a positive direction
    intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED_UP);
    

  }

  /** will lower the intake down to the ground */
  public void lowerIntake(){
    //SparkMaxLimitSwitch limit = intakeRotationMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    // intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED);
    // //used this for help https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Soft%20Limits/src/main/java/frc/robot/Robot.java 
    // intakeRotationMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // intakeRotationMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) 0.0);
    // intakeRotationMotorRaised = false;
    loweringIntake = true;

    //Lowering the intake is a negative direction
    intakeRotationMotor.set(-Constants.INTAKE_ROTATION_MOTOR_SPEED_DOWN);

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

    //intakeRotationSpeedDisplay.setDouble(intakeRotationMotor.get());//updates the display for the intake rotation motor speed
    //manage raise and lower intake
    //raise intake
    if(raisingIntake){
      
      if(!upperLimitSwitch.get() || (getEncoderPosition() >= Constants.INTAKE_UPPER_ENCODER_VALUE)){
        raisingIntake = false;
        intakeRotationMotorRaised = true;
        intakeRotationMotor.set(0.0);
      }
      
    }
    else if(loweringIntake){
      
      if(!lowerLimitSwitch.get() || (getEncoderPosition() <= Constants.INTAKE_LOWER_ENCODER_VALUE)){
        loweringIntake = false;
        intakeRotationMotorRaised = false;
        intakeRotationMotor.set(0.0);
        setSpeedIntake(Constants.INTAKE_MOTOR_SPEED);
      }
    }

    //used for testing purposes
    // if(raisingIntake){
    //   fakeEncoderPosition -= 0.05;
    // }
    // if(loweringIntake){
    //   fakeEncoderPosition += 0.05;
    // }
  }
}