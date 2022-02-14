// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class turnToAngle extends PIDCommand {

  /** Creates a new turnToAngle. */
  public turnToAngle( Drivetrain drivetrain, double angle) {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0, 0),
        // This should return the measurement
        () -> drivetrain.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          // Use the output here
         drivetrain.drive(0, 0, output);

        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(Constants.turnTolerance, Constants.turnRateTolerance);

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
