package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final class Swerve {

                public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(19.5);
                public static final double wheelBase = Units.inchesToMeters(16.5);

                // nominal (real) divided by fudge factor
                public static final double wheelDiameter = Units.inchesToMeters(4.0/1.04085);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
                public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
                                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
                );

                /* Swerve Profiling Values */
                public static final double maxSpeed = 4.5; // meters per second
                public static final double maxAngularVelocity = 11.5;

                public static final int frontLeftRotationMotorId = 5;
                public static final int frontLeftDriveMotorId = 6;

                public static final int frontRightRotationMotorId = 7;
                public static final int frontRightDriveMotorId = 8;

                public static final int rearLeftRotationMotorId = 3;
                public static final int rearLeftDriveMotorId = 4;

                public static final int rearRightRotationMotorId = 1;
                public static final int rearRightDriveMotorId = 2;

                public static final int frontLeftRotationEncoderId = 11;
                public static final int frontRightRotationEncoderId = 12;
                public static final int rearLeftRotationEncoderId = 10;
                public static final int rearRightRotationEncoderId = 9;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

                public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

        }

        public static final class AutoConstants {

                public static final double kPXController = 0.75;
                public static final double kPYController = 0.75;
                public static final double kPThetaController = 5;

                public static final double kMaxSpeedMetersPerSecond = Swerve.maxSpeed / 4;
                public static final double kMaxAngularSpeedRadiansPerSecond = //
                                Swerve.maxAngularVelocity / 10;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                                new TrapezoidProfile.Constraints(
                                                kMaxAngularSpeedRadiansPerSecond,
                                                kMaxAngularAccelerationRadiansPerSecondSquared);

        }

}
