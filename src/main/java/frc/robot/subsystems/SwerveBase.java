package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveBase extends SubsystemBase {

  /**
   * Subsystem that controls the drivetrain of the robot
   * Handles all the odometry and base movement for the chassis
   */

  /**
   * absolute encoder offsets for the wheels
   * 180 degrees added to offset values to invert one side of the robot so that it
   * doesn't spin in place
   */
  private static final double frontLeftAngleOffset = Units.degreesToRadians(284.117);
  private static final double frontRightAngleOffset = Units.degreesToRadians(285.996 - 61);
  private static final double rearLeftAngleOffset = Units.degreesToRadians(137 + 134);
  private static final double rearRightAngleOffset = Units.degreesToRadians(232.229 + 2);

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor can ID
   * rotation motor can ID
   * external CANCoder can ID
   * measured CANCoder offset
   */

  private final SwerveModule frontLeft = new SwerveModule(
      Swerve.frontLeftDriveMotorId,
      Swerve.frontLeftRotationMotorId,
      Swerve.frontLeftRotationEncoderId,
      frontLeftAngleOffset);

  private final SwerveModule frontRight = new SwerveModule(
      Swerve.frontRightDriveMotorId,
      Swerve.frontRightRotationMotorId,
      Swerve.frontRightRotationEncoderId,
      frontRightAngleOffset);

  private final SwerveModule rearLeft = new SwerveModule(
      Swerve.rearLeftDriveMotorId,
      Swerve.rearLeftRotationMotorId,
      Swerve.rearLeftRotationEncoderId,
      rearLeftAngleOffset);

  private final SwerveModule rearRight = new SwerveModule(
      Swerve.rearRightDriveMotorId,
      Swerve.rearRightRotationMotorId,
      Swerve.rearRightRotationEncoderId,
      rearRightAngleOffset);

  private final AHRS navX;

  /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Swerve.kinematics, new Rotation2d(),
      getModulePositions());

  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  public SwerveBase() {
    navX = new AHRS(SPI.Port.kMXP);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    // odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());

    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    rearLeft.initRotationOffset();
    rearRight.initRotationOffset();

    // reset the measured distance driven for each module
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

    rearRight.getDriveMotor().setInverted(true);
    rearLeft.getDriveMotor().setInverted(true);
    frontRight.getDriveMotor().setInverted(true);
    frontLeft.getDriveMotor().setInverted(true);

    rearRight.getRotationMotor().setInverted(true);
    rearLeft.getRotationMotor().setInverted(true);
    frontRight.getRotationMotor().setInverted(true);
    frontLeft.getRotationMotor().setInverted(true);

  }

  @Override
  public void periodic() {

    // update the odometry every 20ms
    odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putString("Robot pose",
        getPose().toString());
    SmartDashboard.putNumber("navX Heading",
        getHeading().getDegrees());

  }

  /**
   * method for driving the robot
   * Parameters:
   * forward linear value
   * sideways linear value
   * rotation value
   * if the control is field relative or robot relative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */
    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = Swerve.kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states);

  }

  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Swerve.maxSpeed);
    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
    rearRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  // returns an array of SwerveModuleState
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
        new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return states;

  }

  // returns an array of SwerveModulePositions
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeft.getCurrentDistanceMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModulePosition(frontRight.getCurrentDistanceMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModulePosition(rearLeft.getCurrentDistanceMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModulePosition(rearRight.getCurrentDistanceMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return positions;

  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  // reset the current pose to a desired pose
  public void resetOdometry(Pose2d pose) {

    odometry.resetPosition(getHeading(), getModulePositions(), pose);

  }

  // reset the measured distance driven for each module
  public void resetDriveDistances() {

    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

  }

  // get the current heading of the robot based on the gyro
  public Rotation2d getHeading() {

    return Rotation2d.fromDegrees(-navX.getYaw());

  }

  public AHRS getNavX() {
    return navX;
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearRight.stop();
    rearLeft.stop();
  }

}