// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Shooter.SpinShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBollsSmart extends SequentialCommandGroup {
  /** Creates a new ShootBollsSmart. */
  public ShootBollsSmart(Tower tower, Shooter shooter ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //if the shooter is currently spinning, the command executes
      new ConditionalCommand(
        //normal logic
        new ConditionalCommand(
          //if there is a ball there, it runs a modified shoot two balls
          new SequentialCommandGroup( 
            new WaitUntilCommand(shooter::shooterReady),
            new InstantCommand(() ->tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED),tower),
            new WaitUntilCommand(shooter::getShooterSensor),
            new InstantCommand(() ->tower.setSpeedLower(Constants.LOWER_TOWER_SPEED),tower),
            new WaitCommand(.5),
            new WaitUntilCommand(() -> !tower.upperSensor.get()),
            new InstantCommand(() -> tower.stop()),
            new WaitUntilCommand(shooter::shooterReady),
            new InstantCommand(() ->tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED),tower),
            new WaitCommand(.5),
            new InstantCommand(shooter::stopShooter, shooter), 
            new InstantCommand(tower::stop, tower)),
        // if there is not a ball at the top sensor, then it runs a modified shoot one ball
        new SequentialCommandGroup( 
            new WaitUntilCommand(shooter::shooterReady),
            new InstantCommand(() ->tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED),tower),
            new WaitCommand(.5),
            new InstantCommand(shooter::stopShooter, shooter),
            new InstantCommand(tower::stop, tower)),
        //the flag that is used to check if there is a ball in the lower intake 
        () -> tower.isBallHereLower()),
        // does nothing if shooter isn't spinning
        null,
        //checks if the shooter is spinning
        () -> shooter.isShooterSpinning()));
  }
}
