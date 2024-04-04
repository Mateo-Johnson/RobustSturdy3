// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;

public class MoveArm extends Command {
  /** Creates a new MoveArm. */
  PIDController armPID = new PIDController(0.015, 0.00, 0.00);
  public MoveArm() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

  SmartDashboard.putNumber("angl;e", Arm.degrees);
  @SuppressWarnings("unused")
  double move = armPID.calculate(Arm.degrees, 35);


 if (Arm.degrees > 90) {
    Arm.rotateVector(0.25);
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