// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.DriveSubsystem;

public class TurnToAngle extends Command {
  private final DriveSubsystem driveSubsystem;
  public static double initialHeading;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  double tP = 0.0159;
  double tI = 0.0;
  double tD = 0.004;
  PIDController turningPID = new PIDController(tP, tI, tD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    initialHeading = driveSubsystem.getHeading();
    SmartDashboard.putNumber("Heading", initialHeading);
    double turnValue = turningPID.calculate(initialHeading, 90);
    SmartDashboard.putNumber("TurnValue", turnValue);
    driveSubsystem.drive(0, 0, -turnValue, false, true);


  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.setWheelsX();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
