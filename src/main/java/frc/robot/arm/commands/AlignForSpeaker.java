// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// JUST AS A NOTE, THE ARM RESTS AT ABOUT 0.422 ABS ENCODER READING

package frc.robot.arm.commands;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.DriveSubsystem;

public class AlignForSpeaker extends Command {
  /** Creates a new MoveArm. */
  public static double initialHeading;
  private final DriveSubsystem driveSubsystem;

  //PID VALUES FOR MOVING THE ARM
  double aP = 0.01;
  double aI = 0.0;
  double aD = 0.0;

  //PID VALUES BUT THEY'RE DIFFERENT FOR ARM
  double aMP = 0.8;
  double aMI = 0.1;
  double aMD = 0.51;

  //PID VALUES FOR TURNING
  double tP = 0.01145;
  double tI = 0.0000176;
  double tD = 0.00098;

  //CREATE THE ARM AND TURNING PID SYSTEMS
  PIDController armAlignPID = new PIDController(aP, aI, aD);
  PIDController turningPID = new PIDController(tP, tI, tD);
  PIDController armMovePID = new PIDController(aMP, aMI, aMD);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
  NetworkTableEntry tx = table.getEntry("tx");//THE X OFFSET OF THE TARGET IN THE CAMERA VIEW
  NetworkTableEntry ty = table.getEntry("ty"); //THE Y OFFSET OF THE TARGET IN THE CAMERA VIEW
  NetworkTableEntry ta = table.getEntry("ta"); //THE AREA THAT THE TARGET TAKES UP ON THE SCREEN
  NetworkTableEntry tv = table.getEntry("tv"); //GET WHETHER THE LIMELIGHT HAS A TARGET OR NOT (1 OR 0)

  double tX = tx.getDouble(0.0); //SET tx = tX AND SET THE DEFAULT VALUE TO 0
  double tY = ty.getDouble(0.0); //SET ty = tY AND SET THE DEFAULT VALUE TO 0
  double tA = ta.getDouble(0.0); //SET ta = tA AND SET THE DEFAULT VALUE TO 0
  double tV = tv.getDouble(0.0); //SET tv = tV AND SET THE DEFAULT VALUE TO 0

  static AbsoluteEncoder armEncoder = Arm.armEncoder;

  private static final int cyclesPerRotation = 2048;
  private static final int drivenGearTeeth = 60;
  private static final int driveGearTeeth = 15;


  public AlignForSpeaker(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  double knownDistance;
  double knownTA;
  double distance;
  double targetAngle;
  double targetAngleDegrees;
  double currentTA;
  double saveAngle;
  double shooterOffsetDegrees;

  @Override
  public void initialize() {
  }

  public static double armEncoderReading =  (armEncoder.getPosition() - 0.42638435959816) * -1;
  public static double gearRatio = (double) drivenGearTeeth / driveGearTeeth;
  public static int encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio);
  public static double degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution; // Corrected line
  public static double degrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tX = tx.getDouble(0.0); 
    tY = ty.getDouble(0.0); 
    tA = ta.getDouble(0.0); 
    tV = tv.getDouble(0.0);

    armEncoderReading =  (armEncoder.getPosition() - 0.42638435959816) * -1;

    //BASICALLY SAVES THE LAST TARGET ANGLE VALUE BEFORE 0
    if (targetAngleDegrees != 0) { //IF THE ARM HAS AN ANGLE IT WANTS TO REACH
      saveAngle = targetAngleDegrees; //SAVE THAT VALUE
    }


    if (tX >= 0) { //IF WE HAVE A TARGET (IF IT IS OFFSET AT ALL)

      knownDistance = 10.0;  //KNOWN DISTANCE (FEET)
      knownTA = 0.160;  //KNOWN TA AT KNOWN DISTANCE
      currentTA = tA;  //CURRENT TA
  
      distance = calculateDistance(currentTA, knownTA, knownDistance);
  
      shooterOffsetDegrees = 33.1;
      targetAngle = Math.atan((3+1/3) / distance);
      double adjustedArmAngle = targetAngle + Math.toRadians(shooterOffsetDegrees);
      targetAngleDegrees = Math.toDegrees(adjustedArmAngle);

    } else if (tX == 0) { //IF WE DONT HAVE A TARGET (IF THERE'S NO OFFSET (CANT SEE IT))

      targetAngleDegrees = saveAngle; //SET THE CURRENT TARGET ANGLE TO WHATEVER THE LAST TARGET ANGLE WAS

    }


    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("target angle", targetAngleDegrees);

    //ENCODER TRANSLATING
    gearRatio = (double) drivenGearTeeth / driveGearTeeth;
    encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio);
    degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution; // Corrected line
    degrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle;
    SmartDashboard.putNumber("arm angle degrees", degrees);

    //PID LOOPS
    // double turnValue = armMovePID.calculate(degrees, targetAngleDegrees);
    double armValue = armAlignPID.calculate(tY, 2);
    double turnValue1 = turningPID.calculate(tX, 0);

    //MOVE THE ARM TO THE SPECIFIC VALUE ABOVE THE APRILTAG
    PIDMoveArm(armValue);
    //ALIGN THE DRIVETRAIN TO THE APRILTAG
    driveSubsystem.drive(0, 0, turnValue1, false, true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.leftArm.set(0);
    Arm.rightArm.set(0);
  }

  //FUNCTION FOR MOVING THE ARM
  public void PIDMoveArm(double angle) {
    Arm.leftArm.set(-angle * 2);
    Arm.rightArm.set(angle * 2);
  }

  public static double calculateDistance(double currentTA, double knownTA, double knownDistance) {
    // Calculate the ratio of current and known tA values
    double tA_ratio = currentTA / knownTA;

    // Adjust the distance based on the ratio
    double distance = knownDistance / tA_ratio;

    return distance;
}




                                              //METHODS FOR ESTIMATING ARM ANGLE//
//-----------------------------------------------------------------------------------------------------//
 //IF WE KNOW THE BEST FOR WHEN WE'RE CLOSE AND THE BEST FOR WHEN WE'RE FAR AWAY, THEN WE KNOW THE IN-BETWEEN
public static double mapLinearDistance(double distance) {
  //CONSTANTS for the two points
  double x1 = 0.0; //DISTANCE = 0
  double y1 = 0.0; //ARM ANGLE = 0 degrees

  double x2 = 10.0; // distance = 10
  double y2 = 90.0; // arm angle = 90 degrees

  // Linear interpolation formula
  double armAngle = y1 + ((distance - x1) / (x2 - x1)) * (y2 - y1);

  return armAngle;
}


public static double mapDistanceToVerticalOffset(double distance) { //CALCULATE VERTICAL OFFSET BY USING DISTANCE
  //CONSTANTS FOR THE TWO POINTS
  double x1 = 0.0;  // DISTANCE = 0
  double y1 = 20.0; // WORKING VERTICAL OFFSET AT 0

  double x2 = 10.0; // DISTANCE = 10
  double y2 = 5.0;  // WORKING VERTICAL OFFSET AT 10

  // Linear interpolation formula for vertical offset
  double verticalOffset = y1 + ((distance - x1) / (x2 - x1)) * (y2 - y1);

  return verticalOffset;
}

public static double estimateVerticalOffset(double currentDistance, double knownDistance, double knownOffset) {
  // Calculate the ratio of current and known distances
  double distanceRatio = currentDistance / knownDistance;

  // Estimate the vertical offset based on the ratio
  double estimatedOffset = knownOffset * distanceRatio;

  return estimatedOffset;

}

public static double averageVerticalOffset(double currentDistance, double knownDistance, double knownOffset) {

  double avgOffset = (estimateVerticalOffset(currentDistance, knownDistance, knownOffset) + mapDistanceToVerticalOffset(currentDistance)) / 2;
  return avgOffset;

}

 //--------------------------------------------------------------------------------------------------//




  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}