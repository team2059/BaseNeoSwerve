// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveBase;

public class AutoAlignCmd extends CommandBase {
  private final Limelight limelight;
  private final SwerveBase swerveBase;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  double rotationSpeed;

  /** Creates a new AutoAlignCmd. */
  public AutoAlignCmd(Limelight limelight, SwerveBase swerveBase) {
    this.limelight = limelight;
    this.swerveBase = swerveBase;
    addRequirements(limelight, swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.hasTargets()) {
      // Calculate angular turn power
      rotationSpeed = turnController.calculate(limelight.getTargetYaw(), 0);
      SmartDashboard.putNumber("rotation speed", rotationSpeed);

    } else {
      rotationSpeed = 0;
    }
    swerveBase.drive(0, 0, rotationSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
