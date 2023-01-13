// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Swerve;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver;

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 4;

  /* Driver Buttons */
  private final JoystickButton zeroGyro;
  private final JoystickButton alignWithTarget;
  private final JoystickButton followTag;
  private final JoystickButton autoBalance;

  /* Subsystems */
  private final SwerveBase swerveBase;
  private final Limelight limelight;

  public Joystick getDriver() {
    return driver;
  }

  public SwerveBase getSwerveSubsytem() {
    return swerveBase;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new Joystick(0);
    zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    alignWithTarget = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    followTag = new JoystickButton(driver, XboxController.Button.kB.value);
    autoBalance = new JoystickButton(driver, XboxController.Button.kX.value);
    swerveBase = new SwerveBase();
    limelight = new Limelight();
    swerveBase.setDefaultCommand(
        new TeleopSwerve(
            swerveBase,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> !driver.getRawButton(XboxController.Button.kLeftBumper.value)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), anxd then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // zeroGyro.whenPressed(new InstantCommand(() -> swerveBase.getNavX().reset()));
    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().reset()));

    // followTag.whenPressed(new SequentialChaseTagCmd(swerveBase, limelight));
    followTag.onTrue(new SequentialChaseTagCmd(swerveBase, limelight));

    // alignWithTarget.whileHeld(new VisionAlignCmd(limelight, swerveBase));

    alignWithTarget.whileTrue(new VisionAlignCmd(limelight, swerveBase));

    autoBalance.onTrue(new AutoBalanceCmd(swerveBase));

  }

  public Trajectory jsonToTrajectory(String filename, boolean resetOdometry) {

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return trajectory;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
      System.out.println("Unable to read from file " + filename);
    }
    return null;

  }

  public PathPlannerTrajectory getPathPlannerTrajectory(String pathName, double maxVelocity, double maxAcceleration) {
    PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration);
    PathPlannerTrajectory examplPathPlannerTrajectory = PathPlanner.loadPath(pathName, constraints, false);
    return examplPathPlannerTrajectory;
  }

  public Command getPathPlannerCommand(PathPlannerTrajectory trajectory) {

    PIDController xController = new PIDController(3.5, 0, 0);
    PIDController yController = new PIDController(3.5, 0, 0);
    PIDController thetaController = new PIDController(
        1.5, 0.0, 0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        trajectory,
        swerveBase::getPose,
        Swerve.kinematics,
        xController,
        yController,
        thetaController,
        swerveBase::setModuleStates,
        swerveBase);

    return command;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    PathPlannerTrajectory trajectory = getPathPlannerTrajectory("NewPath", 1, 2);
    System.out.println("degrees pose trajectory = " + trajectory.getInitialPose().getRotation().getDegrees());
    Command ppCommand = getPathPlannerCommand(trajectory);

    return new SequentialCommandGroup(

        new InstantCommand(() -> swerveBase.resetOdometry(trajectory.getInitialPose())),
        ppCommand,
        new InstantCommand(() -> swerveBase.stopModules()));
  }
}
