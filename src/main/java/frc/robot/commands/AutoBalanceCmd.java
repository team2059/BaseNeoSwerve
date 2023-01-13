// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBase;

public class AutoBalanceCmd extends CommandBase {
  SwerveBase swerveBase;
  double roll;
  double error;
  double driveSpeed;
  double kDriveP;
  boolean haveIBeenTilted = false;

  /** Creates a new AutoBalanceCmd. */
  public AutoBalanceCmd(SwerveBase swerveBase) {
    this.swerveBase = swerveBase;
    addRequirements(swerveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = swerveBase.getNavX().getRoll();
    if (Math.abs(roll) > 5) {
      haveIBeenTilted = true;
    }
    System.out.println("roll" + roll);
    error = 0 - roll;
    System.out.println("error" + error);
    swerveBase.drive(0.45, 0, 0, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE BALANCING");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (haveIBeenTilted && Math.abs(error) < 2.5);
  }
}
