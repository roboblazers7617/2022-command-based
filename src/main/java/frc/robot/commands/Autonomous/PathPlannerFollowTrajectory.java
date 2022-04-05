// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerFollowTrajectory extends SequentialCommandGroup {
  /** Creates a new PathPlannerFollowTrajectory. */
  public PathPlannerFollowTrajectory(Drivetrain drivetrain, String trajectoryJSON, double maxVelocity, double maxAccel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PathPlannerTrajectory trajectoryPath = PathPlanner.loadPath(trajectoryJSON, maxVelocity, maxAccel);

    // MAX_ROTATION? Shouldn't this be more than 10?
   //TrapezoidProfile.Constraints anglePIDProfile = new TrapezoidProfile.Constraints(Constants.MAX_ROTATION,Constants.MAX_ROTATION_CHANGE );

    //var thetaController = new ProfiledPIDController(0,0,0, anglePIDProfile);
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPMecanumControllerCommand mecanumContollerCommand = new PPMecanumControllerCommand(
        trajectoryPath, 
        drivetrain::getPose,
        drivetrain.getKinematics(),
        drivetrain.getXPID(),
        drivetrain.getYPID(),
        drivetrain.getThetaPID(),
        Constants.MAX_VELOCITY, 
        drivetrain::setSpeeds, 
        drivetrain);

        drivetrain.resetOdometry(trajectoryPath.getInitialPose()); // Is this needed?
    
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(trajectoryPath.getInitialPose())), // should this be getIntialState or InitialPose?
      new InstantCommand(() -> drivetrain.getThetaPID().reset(drivetrain.getPose().getRotation().getRadians())),
      mecanumContollerCommand,
      new InstantCommand(() ->drivetrain.drive(0, 0, 0)));
  }
}
