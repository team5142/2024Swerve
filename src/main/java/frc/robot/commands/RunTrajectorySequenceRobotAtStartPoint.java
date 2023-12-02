// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequenceRobotAtStartPoint extends SequentialCommandGroup {

  PathPlannerTrajectory trajectoryPath;
  boolean isReversed = false;

  /** Creates a new RunTrajectorySequenceRobotAtStartPoint.
   * This command reads a PathPlanner trajectory from a file specified by name in the first parameter,
   * optionally applies max velocity and max acceleration restrictions as well as reverse if needed,
   * and runs the adjusted trajectory via AutonomousTrajectoryRioCommand command.
   */
  public RunTrajectorySequenceRobotAtStartPoint(String trajectory, double maxVelocity, double maxAcceleration, boolean reversed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Read the trajectory from a file
    trajectoryPath = PathPlanner.loadPath(trajectory, new PathConstraints(maxVelocity, maxAcceleration), reversed);

    addCommands(
      //new InstantCommand(RobotContainer.driveSubsystem::zeroDriveEncoders),
      new PrintCommand("****Starting trajectory****"),
      //new WaitCommand(0.4),
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYawForTrajectory(trajectoryPath.getInitialHolonomicPose().getRotation().getDegrees()) ),
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialHolonomicPose()  ) ),
      //new PrintCommand(
      //  "START IX:" + trajectoryPath.getInitialPose().getX()+
      //  " IY:" + trajectoryPath.getInitialPose().getY()+
      //  " IA:" + trajectoryPath.getInitialPose().getRotation().getDegrees()
      //  ),  // Set the initial pose of the robot to the one in a trajectory
      new AutonomousTrajectoryRioCommand(trajectoryPath), // Run a trajectory
      new InstantCommand( () -> RobotContainer.imuSubsystem.restoreYawAfterTrajectory()),
      new PrintCommand("****End trajectory****")
    );
  }

  public RunTrajectorySequenceRobotAtStartPoint(String trajectory) {

    this(trajectory, false);
    System.out.println("*** Run trajectory non-reversed"+ trajectory);
  }

  /**
   * Run trajectory from a saved PathPlanner trajectory file with the maximum velocity and acceleration
   * defined in Constant file.
   * @param trajectory - name of the trajectory file without extension
   * @param reversed - whether the trajectory should run in reverse
   */
  public RunTrajectorySequenceRobotAtStartPoint(String trajectory, boolean reversed) {

    //this(trajectory, 0.5, 0.05, reversed);
    this(trajectory, Constants.SwerveChassis.MAX_VELOCITY, Constants.SwerveChassis.MAX_ACCELERATION, reversed);
    System.out.println("*** Run trajectory "+ trajectory+" reversed:"+reversed+" with max velocity and acceleration");
  }

}
