// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.DriveSubsystem;

public class two_silly extends Command {
  /** Creates a new man idk. */
  private final DriveSubsystem driveSubsystem;
  public static Pose2d currentPosition;
  public static double currentPositionX;
  public static double currentPositionY;

  public static Pose2d originalPos;
  public static double originalPosX;
  public static double originalPosY;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  double tX = tx.getDouble(0.0); //SET tx = tX AND SET THE DEFAULT VALUE TO 0
  double tY = ty.getDouble(0.0); //SET ty = tY AND SET THE DEFAULT VALUE TO 0
  double tA = ta.getDouble(0.0); //SET ta = tA AND SET THE DEFAULT VALUE TO 0
  double tV = tv.getDouble(0.0); //SET tv = tV AND SET THE DEFAULT VALUE TO 0

  public two_silly(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    currentPosition = DriveSubsystem.lmao;
  }

  double dP = 0.8;
  double dI = 0.070568344;
  double dD = 0.3;

  double aP = 0.01;
  double aI = 0.0;
  double aD = 0.0;
  
  PIDController lateralPID = new PIDController(dP, dI, dD);
  // PIDController armAlignPID = new PIDController(aP, aI, aD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPosition = driveSubsystem.swerveDrivePoseEstimator.getEstimatedPosition();
    originalPosX = DriveSubsystem.x;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tX = tx.getDouble(0.0); //SET tx = tX AND SET THE DEFAULT VALUE TO 0
    double tY = ty.getDouble(0.0); //SET ty = tY AND SET THE DEFAULT VALUE TO 0
    double tA = ta.getDouble(0.0); //SET ta = tA AND SET THE DEFAULT VALUE TO 0
    double tV = tv.getDouble(0.0); //SET tv = tV AND SET THE DEFAULT VALUE TO 0

    SmartDashboard.putNumber("orange", tX); //PUT THE CURRENT X POSITION ON SMARTDASHBOARD
    SmartDashboard.putNumber("alcohol", tA); //PUT THE CURRENT X POSITION ON SMARTDASHBOARD
    double moveValue = lateralPID.calculate(tX, -4); //CALCULATE PID CONSTANTS TO MAKE THE CURRENT POSITION = ORIGINAL POS X
    double moveValue2 = lateralPID.calculate(tA, 1); //CALCULATE PID CONSTANTS TO MAKE THE CURRENT POSITION = ORIGINAL POS X
    SmartDashboard.putNumber("TurnValue", moveValue);
    driveSubsystem.drive(-moveValue2, -moveValue, 0, false, true);
    // double armValue = armAlignPID.calculate(tY, 8);
    // PIDMoveArm(armValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setWheelsX();
  }

    public void PIDMoveArm(double angle) {
    Arm.leftArm.set(-angle * 2);
    Arm.rightArm.set(angle * 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


