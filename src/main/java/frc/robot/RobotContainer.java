// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);
  private final Drivetrain drivetrain = new Drivetrain();
  private final AutoCommand autoCommand = new AutoCommand();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(getTeleOpDrive());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton speedButton = new JoystickButton(controller, Constants.SPEED_ADJUSTOR_TRIGGER);
    speedButton.whenPressed(new SpeedAdjustor(drivetrain));
    
  }

  public Command getTeleOpDrive(){
    return new TeleOpDrive(drivetrain,() -> controller.getLeftX(), () -> controller.getLeftY(), () -> controller.getRightX());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig tConfig = new TrajectoryConfig(Units.feetToMeters(Constants.MAX_VELOCITY), Units.feetToMeters(Constants.MAX_ACCELERATION));
    tConfig.setKinematics(drivetrain.getKinematics());
    PathPlannerTrajectory test = PathPlanner.loadPath("test Path", Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION);
    MecanumControllerCommand testingFunctionality = new MecanumControllerCommand(test, 
    drivetrain :: getPose, 
    drivetrain.getFeedforward(),
    drivetrain.getKinematics(), 
    drivetrain.getXPID(), 
    drivetrain.getYPID(), 
    drivetrain.getThetaPID(), 
    Units.feetToMeters(Constants.MAX_VELOCITY),
    drivetrain.getLeftFrontPID(),
    drivetrain.getLeftBackPID(),
    drivetrain.getRightFrontPID(),
    drivetrain.getRightBackPID(), 
    drivetrain :: getSpeed,
    drivetrain::setMotorVoltages, 
    drivetrain);
    drivetrain.resetOdometry(test.getInitialPose());
    return testingFunctionality.andThen(() -> drivetrain.drive(0, 0, 0));
  }
}