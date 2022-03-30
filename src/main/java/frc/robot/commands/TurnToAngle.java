  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(Drivetrain drivetrain, double relativeAngle) {
    super( 
        // The controller that the command will use
        new PIDController(Constants.DRIVETRAIN_ROTATIONAL_KP, Constants.DRIVETRAIN_KI, Constants.DRIVETRAIN_KD),
        // This should return the measurement
        () -> drivetrain.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> relativeAngle-drivetrain.getAngle(),
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.drive(0, 0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
