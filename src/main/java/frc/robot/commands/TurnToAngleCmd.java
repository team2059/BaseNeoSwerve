// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBase;

public class TurnToAngleCmd extends CommandBase {
  SwerveBase swerveBase;
  double angleDegrees;
  final double ANGULAR_P = 0.066;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  double rotationSpeed;

  /** Creates a new TurnToAngleCmd. */
  public TurnToAngleCmd(SwerveBase swerveBase, double angleDegrees) {
    this.swerveBase = swerveBase;
    this.angleDegrees = angleDegrees;
    addRequirements(swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("set point", angleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationSpeed = turnController.calculate(swerveBase.getHeading().getDegrees(), angleDegrees);
    swerveBase.drive(0, 0, rotationSpeed, true);

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