// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commands.setpoints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;

public class Looking_Pos extends Command {
  /** Creates a new SpecifiedAngle. */
  double currentArmPos;
  double targetArmPos;
  double armValue;

  PIDController armPID = Arm.armPID;

  public Looking_Pos() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentArmPos = Arm.degrees;
    targetArmPos = 60;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentArmPos = Arm.degrees;

    armValue = armPID.calculate(currentArmPos, targetArmPos);
    Arm.rotateVector(armValue);
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
