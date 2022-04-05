// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_WHEEL_PORT,MotorType.kBrushless);
 
  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();
 
  private final MecanumDrive drivetrain;
 
  private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(Units.inchesToMeters(10.375),Units.inchesToMeters(10.375)), 
            new Translation2d(Units.inchesToMeters(10.375),Units.inchesToMeters(-10.375)), 
            new Translation2d(Units.inchesToMeters(-10.375),Units.inchesToMeters(10.375)), 
            new Translation2d(Units.inchesToMeters(-10.375),Units.inchesToMeters(-10.375)));

            
  private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, getAngle2d());
  
  private Pose2d pose;
  
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DRIVETRAIN_KS, Constants.DRIVETRAIN_KV, Constants.DRIVETRAIN_KA);
  
  private final PIDController leftFrontPID = new PIDController(Constants.WHEEL_KP, Constants.WHEEL_KI, Constants.WHEEL_KD);
  private final PIDController rightFrontPID = new PIDController(Constants.WHEEL_KP, Constants.WHEEL_KI, Constants.WHEEL_KD);
  private final PIDController leftBackPID = new PIDController(Constants.WHEEL_KP, Constants.WHEEL_KI, Constants.WHEEL_KD);
  private final PIDController rightBackPID = new PIDController(Constants.WHEEL_KP, Constants.WHEEL_KI, Constants.WHEEL_KD);
  
  public final TrapezoidProfile.Constraints anglePIDProfile = new TrapezoidProfile.Constraints(Constants.MAX_ROTATION,Constants.MAX_ROTATION_CHANGE );
  private final PIDController xPID = new PIDController(Constants.DRIVETRAIN_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD);
  private final PIDController yPID = new PIDController(Constants.DRIVETRAIN_ROTATIONAL_KP, Constants.DRIVETRAIN_ROTATIONAL_KI, Constants.DRIVETRAIN_KD);
  
  private final ProfiledPIDController thetaPID = new ProfiledPIDController(1, 0, 0, anglePIDProfile); // Jill Changed the first value to 1

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    ///Original Inversions
    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
    drivetrain = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    
    drivetrain.setMaxOutput(Constants.HIGH_GEAR);
    gyro.calibrate();
    gyro.reset();

    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);

    //// JILL Commented out
   /*
    leftFrontEncoder.setPositionConversionFactor(Constants.WHEEL_GEAR_RATIO*Constants.PULSE_RELATIVE_ENCODER);
    leftBackEncoder.setPositionConversionFactor(Constants.WHEEL_GEAR_RATIO*Constants.PULSE_RELATIVE_ENCODER);
    rightFrontEncoder.setPositionConversionFactor(Constants.WHEEL_GEAR_RATIO*Constants.PULSE_RELATIVE_ENCODER);
    rightBackEncoder.setPositionConversionFactor(Constants.WHEEL_GEAR_RATIO*Constants.PULSE_RELATIVE_ENCODER);  
  */
    leftFrontEncoder.setPositionConversionFactor(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION);
    leftBackEncoder.setPositionConversionFactor(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION);
    rightFrontEncoder.setPositionConversionFactor(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION);
    rightBackEncoder.setPositionConversionFactor(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getAngle2d(), getSpeed());

    var translation = odometry.getPoseMeters().getTranslation();

    ShuffleboardInfo.getInstance().getGyroEntry().setDouble(getAngle());
    ShuffleboardInfo.getInstance().getDrivetrainLeftFrontMotorEntry().setDouble(leftFrontEncoder.getPosition());
    ShuffleboardInfo.getInstance().getDrivetrainRightFrontMotorEntry().setDouble(rightFrontEncoder.getPosition());
    ShuffleboardInfo.getInstance().getDrivetrainLeftRearMotorEntry().setDouble(leftBackEncoder.getPosition());
    ShuffleboardInfo.getInstance().getDrivetrainRightRearMotorEntry().setDouble(rightBackEncoder.getPosition());
    ShuffleboardInfo.getInstance().getDrivetrainXPoseEntry().setDouble(translation.getX());
    ShuffleboardInfo.getInstance().getDrivetrainYPoseEntry().setDouble(translation.getY());

  }


  public void drive(double ySpeed, double xSpeed, double zRotation){
    drivetrain.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  public void setMaxSpeed(double speed) {
    drivetrain.setMaxOutput(speed);
  }

  /******************************************************* */
  /** Functions to get and set the wheel speeds            */
  /******************************************************* */
  public void setSpeeds(double leftFrontSpeed, double rightFrontSpeed, double leftBackSpeed, double rightBackSpeed){
    leftFrontMotor.set(leftFrontSpeed);
    rightFrontMotor.set(rightFrontSpeed);
    leftBackMotor.set(leftBackSpeed);
    rightBackMotor.set(rightBackSpeed);
  }

  public MecanumDriveWheelSpeeds getSpeed(){
    return new MecanumDriveWheelSpeeds(
    
    // Why is this divided by 60?
    leftFrontEncoder.getVelocity()/60,
    rightFrontEncoder.getVelocity()/60,
    leftBackEncoder.getVelocity()/60,
    rightBackEncoder.getVelocity()/60

    // JILL Commented out
    /*Constants.WHEEL_GEAR_RATIO*leftFrontEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60, 
    Constants.WHEEL_GEAR_RATIO*rightFrontEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60, 
    Constants.WHEEL_GEAR_RATIO*rightBackEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60, 
    Constants.WHEEL_GEAR_RATIO*leftBackEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60
    */
    );
  }


  /** Source: WPILIB MecanumDrivePoseEstimator project example code */
  public void setSpeeds(MecanumDriveWheelSpeeds wheelSpeeds){

    final double frontLeftFeedForward = feedforward.calculate(wheelSpeeds.frontLeftMetersPerSecond);
    final double frontRightFeedForward = feedforward.calculate(wheelSpeeds.frontRightMetersPerSecond);
    final double rearLeftFeedForward = feedforward.calculate(wheelSpeeds.rearLeftMetersPerSecond);
    final double rearRightFeedForward = feedforward.calculate(wheelSpeeds.rearRightMetersPerSecond);

    final double frontLeftOutput = leftFrontPID.calculate(leftFrontEncoder.getVelocity(), wheelSpeeds.frontLeftMetersPerSecond);
    final double frontRightOutput = rightFrontPID.calculate(rightFrontEncoder.getVelocity(), wheelSpeeds.frontRightMetersPerSecond);
    final double rearLeftOutput = leftBackPID.calculate(leftBackEncoder.getVelocity(), wheelSpeeds.rearLeftMetersPerSecond);
    final double rearRightOutput = rightBackPID.calculate(rightBackEncoder.getVelocity(), wheelSpeeds.rearRightMetersPerSecond);

    leftFrontMotor.setVoltage(frontLeftOutput + frontLeftFeedForward);
    rightFrontMotor.setVoltage(frontRightOutput + frontRightFeedForward);
    leftBackMotor.setVoltage(rearLeftOutput + rearLeftFeedForward);
    rightBackMotor.setVoltage(rearRightOutput + rearRightFeedForward);

  }
  
  public void setMotorVoltages(MecanumDriveMotorVoltages volts){
    leftFrontMotor.setVoltage(volts.frontLeftVoltage);
    rightFrontMotor.setVoltage(volts.frontRightVoltage);
    leftBackMotor.setVoltage(volts.rearLeftVoltage);
    rightBackMotor.setVoltage(volts.rearRightVoltage);
  }

  /****************************************************** */
  /** Functions for gyro                                  */
  /****************************************************** */
  public double getAngle(){
    return -gyro.getAngle();
  }

  public void resetGyro(){
    gyro.reset();
  }
  
  /**************************************************** */
  /* Functions for odometry and path following          */
  /**************************************************** */
  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public MecanumDriveKinematics getKinematics(){
    return kinematics;
  }
  public PIDController getXPID() {
      return xPID;
  }
  public PIDController getYPID() {
      return yPID;
  }
  public ProfiledPIDController getThetaPID() {
      return thetaPID;
  }

  private Rotation2d getAngle2d(){
    return Rotation2d.fromDegrees(-getAngle());
  }

  public Pose2d getPose(){
    return pose;
  }

  public PIDController getLeftFrontPID (){
    return leftFrontPID;
  }

  public PIDController getRightFrontPID (){
    return rightFrontPID;
  }

  public PIDController getLeftBackPID (){
    return leftBackPID;
  }

  public PIDController getRightBackPID (){
    return rightBackPID;
  }


  public void resetOdometry(Pose2d pose){

    //odometry.resetPosition(pose, gyro.getRotation2d()); // Will this return the correct postivie/negative value?

    // Jill commented out resetting the encoders
    //leftFrontEncoder.setPosition(0);
    //leftBackEncoder.setPosition(0);
    //rightFrontEncoder.setPosition(0);
    //rightBackEncoder.setPosition(0);
  }


  /****************************************************** */
  /**  Encoder functions                                  */
  /****************************************************** */
  public void resetEncoders(){
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }

  public double getAverageEncoderPosition(){
    return (leftFrontEncoder.getPosition()+rightFrontEncoder.getPosition()+leftBackEncoder.getPosition()+rightBackEncoder.getPosition())/4;
  }

  // Don't know if this works
  public double getAverageStrafePosition(){
    return (leftFrontEncoder.getPosition()+rightBackEncoder.getPosition())/2;
  }
  public double getEncoderDistance()
  {
     return getAverageEncoderPosition();
  }

  // Don't know if this works
  public double getEncoderStrafeDistance()
  {

    return getAverageStrafePosition();
  }

  // Don't think this works
  public double[] getRobotTranslation(){
    double leftFrontWheelMovement = leftFrontEncoder.getPosition();
    double leftBackWheelMovement = leftBackEncoder.getPosition();
    double rightFrontWheelMovement = rightFrontEncoder.getPosition();
    double rightBackWheelMovement = rightBackEncoder.getPosition();
    double[] distanceTraveled = new double[2];
    //forward/backward distance
  //distanceTraveled[0] = leftFrontWheelMovement / Math.sqrt(2) + leftBackWheelMovement / Math.sqrt(2) + rightFrontWheelMovement / Math.sqrt(2) + rightBackWheelMovement / Math.sqrt(2);
    distanceTraveled[1] = (leftFrontWheelMovement + rightFrontWheelMovement)/2;  
  //left/right distance
    distanceTraveled[0] = rightFrontWheelMovement / Math.sqrt(2) - leftFrontWheelMovement / Math.sqrt(2) + leftBackWheelMovement / Math.sqrt(2) - rightBackWheelMovement / Math.sqrt(2);
    return distanceTraveled;
    
  }


  public void setBrakeMode(String mode){
    if(mode.equals("brake") || mode.equals("Brake"))
    {
      leftFrontMotor.setIdleMode(IdleMode.kBrake);
      rightFrontMotor.setIdleMode(IdleMode.kBrake);
      leftBackMotor.setIdleMode(IdleMode.kBrake);
      rightBackMotor.setIdleMode(IdleMode.kBrake);
    }

    if(mode.equals("coast") || mode.equals("Coast"))
    {
      leftFrontMotor.setIdleMode(IdleMode.kCoast);
      rightFrontMotor.setIdleMode(IdleMode.kCoast);
      leftBackMotor.setIdleMode(IdleMode.kCoast);
      rightBackMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  }

