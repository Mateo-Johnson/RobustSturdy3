// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.vision.Vision;

public class Amp extends Command {
    private final DriveSubsystem driveSubsystem;

    double distance;
    double tY;
    double tX;
    double targetID;
    double deadzone = 3;
    double arm;
    double yPID;
    double xPID;

    PIDController armPID = Arm.armPID;
    PIDController lateralPID = new PIDController(0.0159, 0.00, 0.004);
  /** Creates a new silly. */
  public Amp(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  tY = Vision.a_tY; //THE Y OFFSET OF THE TARGET
  tX = Vision.a_tX; //THE Y OFFSET OF THE TARGET
  targetID = (int) Vision.a_targetID; //SET TARGET ID EQUAL TO THE INT VALUE FROM VISION.JAVA BUT CAST IT TO INT
  distance = Vision.calculateDistance(tY, Arm.degrees, targetID, "amp");

  if (tX <= -deadzone && tX >= deadzone) {

    arm = armPID.calculate(Arm.degrees, 90);
    driveSubsystem.drive(0.1, 0, 0, false, true);

  } else if (tX >= -deadzone || tX <= deadzone) {

    arm = armPID.calculate(Arm.degrees, 12); //PLACEHOLDER VALUE
    Arm.rotateVector(arm);
    yPID = lateralPID.calculate(tX, 0);
    xPID = lateralPID.calculate(distance, 2);
    driveSubsystem.drive(xPID, yPID, 0, false, true);
  }
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