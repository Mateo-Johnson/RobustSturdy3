// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;

public class MoveArmBackwards extends Command {
  /** Creates a new MoveArmBackwards. */
  public MoveArmBackwards() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 if (Arm.degrees > 3) {
    Arm.rotateVector(0.1);
    } else {
      Arm.rotateVector(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.leftArm.stopMotor();
    Arm.rightArm.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}