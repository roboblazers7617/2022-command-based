// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final Intake intake = new Intake();
  private final Tower tower = new Tower();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(getTeleOpDrive());
    Shuffleboard.getTab("Debug").add("ToggleIntake", new ToggleIntake(intake));
    Shuffleboard.getTab("Debug").add("ToggleIntakeReverse", new ToggleIntakeReverse(intake));
    Shuffleboard.getTab("Debug").add("ActivateTower", new ActivateTower(tower));
    Shuffleboard.getTab("Debug").add("StopTower", new StopTower(tower));
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
    return autoCommand;
  }
}
