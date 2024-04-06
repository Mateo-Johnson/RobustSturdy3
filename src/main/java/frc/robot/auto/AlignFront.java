// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.Vision;

public class AlignFront extends Command {
  /** Creates a new AlignFront. */
  PIDController tPID = Auto.turnPID;
  double tX;
  double tY;
  boolean tV;
  double twurn;

  public AlignFront() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tX = Vision.f_tX;
    tY = Vision.f_tY;
    tV = Vision.f_tV;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tX = Vision.f_tX;
    tY = Vision.f_tY;
    tV = Vision.f_tV;

    twurn = tPID.calculate(tX, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
