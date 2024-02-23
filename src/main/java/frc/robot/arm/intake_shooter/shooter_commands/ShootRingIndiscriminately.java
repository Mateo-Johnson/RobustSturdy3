// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.intake_shooter.Intake_shooter;

public class ShootRingIndiscriminately extends Command {
  /** Creates a new silly. */
  public ShootRingIndiscriminately() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake_shooter.runOuttake();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake_shooter.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}