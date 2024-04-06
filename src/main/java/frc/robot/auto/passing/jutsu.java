// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.passing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.arm.Arm;
import frc.robot.auto.Auto;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.vision.Vision;

public class jutsu extends Command {
    private final DriveSubsystem driveSubsystem;
  /** Creates a new jutsu. */
  public jutsu(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
  }

  PIDController tPID = Auto.turnPID;
  PIDController aPID = Arm.armPID;
  boolean armUp;
  double move;
  double turn;
  double tX;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (Arm.degrees < 0 && Arm.degrees > 7) {
    //   armUp = false;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // tX = Vision.f_tX;

    // if (armUp == false) {

    move = aPID.calculate(Arm.degrees, 85);
    // armUp = true;
    
    // } else if (armUp == true) {
    //   if (Vision.f_tV) {
    //     double turn = tPID.calculate(tX, 0);
    //     driveSubsystem.drive(0, 0, -turn, false, true);
    //   }
    // }

  Arm.rotateVector(-move);
  if (Arm.degrees > 80 && Arm.degrees < 90) {
  }

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
