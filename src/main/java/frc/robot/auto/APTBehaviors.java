// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;
import frc.robot.auto.scoring.Auto;
import frc.robot.climber.commands.Climb;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.vision.Vision;

public class APTBehaviors extends Command {
  /** Creates a new Amp. */

  public static double initialHeading;
  private final DriveSubsystem driveSubsystem;

  //VISION SHIT
  public static int targetID;
  double tX;
  double t;
  double tA;
  boolean tV;
  double distance;
  double targetAngle;
  double deadzone;

  static AbsoluteEncoder armEncoder = Arm.armEncoder; 
  private static final int cyclesPerRotation = 2048; //HOW MANY CYCLES PER ROTATION PER ROTATION OF THE ENCODER
  private static final int drivenGearTeeth = 60; //HOW MANY TEETH ON THE SHAFT GEAR
  private static final int driveGearTeeth = 15; //HOW MANY TEETH ON THE MOTOR GEAR
  public static double armEncoderReading =  (armEncoder.getPosition());
  public static double gearRatio = (double) drivenGearTeeth / driveGearTeeth; //CALCULATE THE GEAR RATION
  public static int encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio); //FIGURE OUT THE ENCODER CYCLES PER REVOLUTION OF THE ARM
  public static double degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution; //CALCULATE HOW MANY DEGREES ARE IN A ARM REVOLUTION
  public static double armDegrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle; //CALCULATE THE CURRENT DEGREES

  //PID CONTROLLERS
  double tP = 0.01145;
  double tI = 0.0000176;
  double tD = 0.00098;
  PIDController turningPID = new PIDController(tP, tI, tD);
  double aMP = 0.8;
  double aMI = 0.1;
  double aMD = 0.51;
  PIDController armMovePID = new PIDController(aMP, aMI, aMD);
  double dP = 0.8;
  double dI = 0.070568344;
  double dD = 0.0;
  PIDController lateralPID = new PIDController(dP, dI, dD);
  public static double tPID;
  public static double amPID;
  public static double lPID;
  public static double hPID;



  public APTBehaviors(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetID = (int) Vision.a_targetID; //SET TARGET ID EQUAL TO THE INT VALUE FROM VISION.JAVA BUT CAST IT TO INT
    double tX = Vision.a_tX; //THE X OFFSET OF THE TARGET
    double tY = Vision.a_tY; //THE Y OFFSET OF THE TARGET
    boolean tV = Vision.a_tV; //WHETHER THE LIMELIGHT HAS A TARGET OR NOT



    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {

      if (tV) {

      switch (targetID) {
        case 1:
            //CODE FOR CLOSE HP-STATION
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "hp-station");

            if (tX >= -3 || tX <= 3) {

              lPID = lateralPID.calculate(tX, -5);
              hPID = lateralPID.calculate(distance, 6);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
              
            }

            break;
        case 2:
            //CODE FOR FAR HP-STATION
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "hp-station");

            if (tX >= -deadzone || tX <= deadzone) {

              lPID = lateralPID.calculate(tX, 5);
              hPID = lateralPID.calculate(distance, 6);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
              
            }

            break;
        case 6:
            //CODE FOR BLUE AMP
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 90);
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -deadzone || tX <= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 2);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }
            break;
        case 7:
            //CODE FOR CENTRAL BLUE SPEAKER
            tPID = turningPID.calculate(tX, 0);

            distance = Vision.calculateDistance(tY, armDegrees, targetID, "speaker");
            targetAngle = Auto.calculateAngle(distance);
            amPID = armMovePID.calculate(armDegrees, targetAngle);

            driveSubsystem.drive(0, 0, tPID, false, true);
            Arm.rotateVector(amPID);

            break;
        case 8:
            //CODE FOR NON-CENTRAL BLUE SPEAKER
            tPID = turningPID.calculate(tX, 5);

            distance = Vision.calculateDistance(tY, armDegrees, targetID, "speaker");
            targetAngle = Auto.calculateAngle(distance);
            amPID = armMovePID.calculate(armDegrees, targetAngle);

            driveSubsystem.drive(0, 0, tPID, false, true);
            Arm.rotateVector(amPID);

            break;
        case 14:
            //BLUE STAGE 
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              new Climb();
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -3 || tX <= 3) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 4);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }

            break;
        case 15:
            //BLUE STAGE
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              new Climb();
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -3 || tX <= 3) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 4);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }

            break;
        case 16:
            //BLUE STAGE
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              new Climb();
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -deadzone || tX <= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 4);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }

            break;
        default:
            //SPIN FOR APRILTAG
            driveSubsystem.drive(0, 0, 0.1, false, true);
      }
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {

      switch (targetID) {
        case 3:
            //CODE FOR NON-CENTRAL RED SPEAKER
            tPID = turningPID.calculate(tX, 5);

            distance = Vision.calculateDistance(tY, armDegrees, targetID, "speaker");
            targetAngle = Auto.calculateAngle(distance);
            amPID = armMovePID.calculate(armDegrees, targetAngle);

            driveSubsystem.drive(0, 0, tPID, false, true);
            Arm.rotateVector(amPID);

            break;
        case 4:
            //CODE FOR CENTRAL RED SPEAKER
            tPID = turningPID.calculate(tX, 0);

            distance = Vision.calculateDistance(tY, armDegrees, targetID, "speaker");
            targetAngle = Auto.calculateAngle(distance);
            amPID = armMovePID.calculate(armDegrees, targetAngle);

            driveSubsystem.drive(0, 0, tPID, false, true);
            Arm.rotateVector(amPID);

            break;
        case 5:
            //CODE FOR RED AMP
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 90);
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -deadzone || tX <= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 2);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }
            break;
        case 9:
            //CODE FOR FAR HP-STATION 
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "hp-station");

            if (tX >= -3 || tX <= 3) {

              lPID = lateralPID.calculate(tX, 5);
              hPID = lateralPID.calculate(distance, 6);
              driveSubsystem.drive(hPID, lPID, 0, false, true);

            }

            break;
        case 10:
            //CODE FOR CLOSE HP-STATION
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "hp-station");

            if (tX >= -deadzone || tX <= deadzone) {

              lPID = lateralPID.calculate(tX, -5);
              hPID = lateralPID.calculate(distance, 6);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
              
            }

            break;
        case 11:
            //RED STAGE
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              new Climb();
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -deadzone || tX <= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 4);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }

            break;
        case 12:
            //RED STAGE
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              new Climb();
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -deadzone || tX <= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 4);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }

            break;
        case 13:
            //RED STAGE
            distance = Vision.calculateDistance(tY, armDegrees, targetID, "amp");

            if (tX <= -deadzone && tX >= deadzone) {
              new Climb();
              driveSubsystem.drive(0.1, 0, 0, false, true);
            } else if (tX >= -deadzone || tX <= deadzone) {
              amPID = armMovePID.calculate(armDegrees, 12); //PLACEHOLDER VALUE
              Arm.rotateVector(amPID);
              lPID = lateralPID.calculate(tX, 0);
              hPID = lateralPID.calculate(distance, 4);
              driveSubsystem.drive(hPID, lPID, 0, false, true);
            }

            break;
        default:
            //SPIN FOR APRILTAG
            driveSubsystem.drive(0, 0, 0.1, false, true);
        } 
      }
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
