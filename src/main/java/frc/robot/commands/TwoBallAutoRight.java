// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoRight extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoLeft. */
  public TwoBallAutoRight(Tower tower, Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootOneBoll(shooter, tower),
      new DriveWithEncoders(drivetrain, Constants.AUTO_SPEED, 50),
      new StrafeWithTime(drivetrain, -Constants.AUTO_SPEED, .5),
      new IntakeBall(intake, tower, drivetrain),
      new DriveWithEncoders(drivetrain, Constants.AUTO_SPEED, 20),
      new StrafeWithTime(drivetrain, Constants.AUTO_SPEED, .5),
      new DriveWithEncoders(drivetrain, -Constants.AUTO_SPEED, 60),
      new AdjustOneTower(tower, shooter),
      new ShootOneBoll(shooter, tower)
    );
  }
}
