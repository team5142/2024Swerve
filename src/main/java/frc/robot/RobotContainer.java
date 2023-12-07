// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Devices.Controller;
import frc.robot.commands.AutonomousTrajectoryRioCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.TurnToAngleZeroHeadingCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final IMUSubsystem imuSubsystem = new IMUSubsystem();

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

  public static Controller driveStick;

  public static Controller turnStick;

  public static Controller xboxController;

  public static Controller leftJoystick;

  public static Controller rightJoystick;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  public SwerveAutoBuilder AutoBuilder;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

      // Configure driver interface - binding joystick objects to port numbers
      configureDriverInterface();

      // Configure the trigger bindings
      configureBindings();

      // This command should be for the teleop driving
      // Note that the first three of its parameters are DoubleSupplier, and the last one is a
      // BooleanSupplier
      
      driveSubsystem.setDefaultCommand(
              new DriveManuallyCommand(
                      () -> getDriverXAxis(),
                      () -> getDriverYAxis(),
                      () -> getDriverOmegaAxis(),
                      () -> getDriverFieldCentric()));
                      
    SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", new ZeroHeadingCommand());
        autoChooser.addOption("ThreeMetersForward", new RunTrajectorySequenceRobotAtStartPoint("3MetersForward"));
        autoChooser.addOption("ThreeMetersBackward", new RunTrajectorySequenceRobotAtStartPoint("3MetersBackward"));
        autoChooser.addOption("1MeterSpin180", new RunTrajectorySequenceRobotAtStartPoint("1MeterSpin180"));
  }

  /**
   * Use this method to define your controllers depending on the
   * {@link DriveInterface}
   */
  private void configureDriverInterface() {

      /**
       * We tried driving with a single Logitech joystick that has X,Y and turn axis.
       * However, the team decided to move the turn to the second joystick for now.
       * Note that Controller objects are only used to provide DoubleSupplier methods
       * to the
       * commands that need manual control input (e.g. DriveManuallyCommand)
       */
      driveStick = new Controller(ControllerDevice.DRIVESTICK);
      turnStick = new Controller(ControllerDevice.TURNSTICK);
      xboxController = new Controller(ControllerDevice.XBOX_CONTROLLER);
      leftJoystick = new Controller(ControllerDevice.LEFTJOYSTICK);
      rightJoystick = new Controller(ControllerDevice.RIGHTJOYSTICK);

      System.out.println("Driver interface configured");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      new Trigger(m_exampleSubsystem::exampleCondition)
              .onTrue(new ExampleCommand(m_exampleSubsystem));

      // Schedule `exampleMethodCommand` when the Xbox controller's B button is
      // pressed,
      // cancelling on release.
      // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

      //swerveValuesTesting();

      //trajectoryCalibration();
      //testCalibrateMotorsAndEncodersButtonBindings();
      //swerveValuesTesting();
  }

  /**
   * Swerve control inputs will be supplied as lambdas to the manual drive
   * command.
   * This way it would be easy to be switch between different input devices such
   * as joystick or Xbox controller.
   * In order to do a switch appropriate analog axis of the device would need to
   * be specificed in one of the three
   * methods below.
   */

  /**
   * Logitech joystick returns -1 of Y axis when pushed all the way forward and +1
   * when pushed all the way backwards.
   * Since the robot is oriented on the X axis of the field, pushing joystick
   * forward means robot to go forward
   * along the X axis.
   * The next three commands are used for teleop driving, and they should
   * represent the controller inputs from the field point of view
   * meaning getDriverXAxis() should provide positive value for robot moving
   * towards the opposite side of the field where the other team is
   * located if your team's location is on the left side of the field.
   * getDriverYAxis() positive value will cause the robot move to the left of your
   * team's location on the field.
   * getDriverOmegaAxis() positive value causes the robot to rotate
   * counterclockwise.
   *
   * If you use just one joystick or other controllers, modify the methods below
   * to return the value from the appropriate controler axis.
   *
   * @return
   */
//   private double getDriverXAxis() {
//       return -driveStick.getLeftStickY();
//   }

//   private double getDriverYAxis() {
//       return -driveStick.getLeftStickX();
//   }

//   private double getDriverOmegaAxis() {
//       return -turnStick.getLeftStickOmega();
//   }

private double getDriverXAxis() {
    // return -driveStick.getLeftStickY();
    return -leftJoystick.getLeftStickY();
 }

 private double getDriverYAxis() {
    // return -driveStick.getLeftStickX();
    return -leftJoystick.getLeftStickX();
 }

 private double getDriverOmegaAxis() {
    // return -turnStick.getLeftStickOmega();
    return -rightJoystick.getLeftStickX();
 }

  

  /**
   * If the button is pressed, use robot-centric swerve
   * Otherwise use field-centric swerve (default).
   * Currently it's set to a numbered button on a joystick, but if you use Xbox or
   * similar controller, you may need to modify this
   * On Logitech joystick button #2 seemed to be the most convenient, though we
   * may consider moving it to the drivestick.
   * 
   * @return - true if robot-centric swerve should be used
   */
  private boolean getDriverFieldCentric() {
      return !rightJoystick.getRawButton(OIConstants.robotCentricButton);
  }

  /**
   * 
  * Make sure motors move robot forward with positive power and encoders increase with positive power
  * To enable put a call to this method in configureBindings method
  */
  private void testCalibrateMotorsAndEncodersButtonBindings() {

    new JoystickButton(leftJoystick, 5)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(0)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)));

    new JoystickButton(leftJoystick, 6)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(1)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)));

    new JoystickButton(leftJoystick, 3)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(2)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)));

    new JoystickButton(leftJoystick, 4)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(3)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)));

    new JoystickButton(leftJoystick, 11)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(2)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)));

    new JoystickButton(leftJoystick, 12)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(3)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)));

    new JoystickButton(leftJoystick, 10)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(1)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)));

    new JoystickButton(leftJoystick, 9)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(0)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0)));
  }

  /**
  * Make sure the swerve calculates the right power and angle numbers, or visually test the robot movement
  * To enable put a call to this method in configureBindings method
  */
  private void swerveValuesTesting() { // Field centric numbers applied

    // Move robot to the left
    new JoystickButton(leftJoystick, 3)
        .onTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,0.3,0, true)))
        .onFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Move robot forward
    new JoystickButton(leftJoystick, 4)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0.3,0,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Move robot to the right
    new JoystickButton(leftJoystick, 5)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,-0.3,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Move robot backwards
    new JoystickButton(leftJoystick, 6)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(-0.3,0,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Turn robot counterclockwise
    new JoystickButton(leftJoystick, 7)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,0, 1, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Turn robot clockwise
    new JoystickButton(leftJoystick, 8)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,0, -1, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );
    
    new JoystickButton(rightJoystick, 10)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("SwiggleWiggle"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
  }

  /**
   * Bindings to test simple swerve trajectories done in PathPlanner
   */
  public void trajectoryCalibration() {
      new JoystickButton(leftJoystick, 11)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("1MeterForward"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(leftJoystick, 12)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("1MeterSideways"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(leftJoystick, 9)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("1Meter45Diag"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(leftJoystick, 10)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("MeterStraightTurn90"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(rightJoystick, 11)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("InPlaceTurn90"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(rightJoystick, 10)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("SwiggleWiggle"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(leftJoystick, 7)
              .whileTrue(new ZeroHeadingCommand())
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(leftJoystick, 8)
              .whileTrue(new TurnToAngleZeroHeadingCommand(Rotation2d.fromDegrees(0)))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
      new JoystickButton(rightJoystick, 12)
              .whileTrue(new InstantCommand(RobotContainer.driveSubsystem::testOdometryUpdates));
        

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
