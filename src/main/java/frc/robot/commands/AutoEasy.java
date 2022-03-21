// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoEasy extends SequentialCommandGroup {
  /** Creates a new AutoEasy. */
  public AutoEasy(Drivetrain drivetrain, Shooter shooter, Tower tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveWithEncoders(drivetrain, -Constants.AUTO_SPEED, 40),
                new SpinShooter(shooter),
                new WaitUntilCommand(shooter::shooterReady),
                new RunTower(tower),
                new WaitCommand(2),
                new InstantCommand(() ->tower.setSpeedUpper(0.0)),
                new InstantCommand(() ->tower.setSpeedLower(0.0)), 
                new StopShooter(shooter),
                new DriveWithEncoders(drivetrain, Constants.AUTO_SPEED, Constants.DISTANCE_FROM_FENDER_TO_TAXI)
              );
  }
}
