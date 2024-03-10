// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.Vision;

public class APTBehaviors extends Command {
  /** Creates a new Amp. */
  public static int targetID;
  public APTBehaviors() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetID = (int) Vision.targetID; //SET TARGET ID EQUAL TO THE INT VALUE FROM VISION.JAVA BUT CAST IT TO INT

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {

      switch (targetID) {
        case 1:
            //CODE FOR CLOSE HP-STATION
            //face the apriltag
            //move to a close distance
            //center between the two apriltags

            break;
        case 2:
            //CODE FOR FAR HP-STATION
            //face the apriltag
            //move to a close distance
            //center between the two apriltags

            break;
        case 6:
            //CODE FOR AMP
            //pivot arm up to set position for seeing apriltag
            //center ourselves on the apriltag
            //move forward slowly
            //put arm up at 90 degrees
            //shoot slowly

            break;
        case 7:
            //CODE FOR CENTRAL BLUE SPEAKER
            //face the robot towards speaker
            //calculate distance
            //move arm to right position
            //shoot

            break;
        case 8:
            //CODE FOR NON-CENTRAL BLUE SPEAKER
            //pivot to find the central apriltag
            //(or maybe just shoot using that one)

            break;
        case 14:
            //BLUE STAGE 
            //center yourself 
            //climb

            break;
        case 15:
            //BLUE STAGE
            //center yourself 
            //climb

            break;
        case 16:
            //BLUE STAGE
            //center yourself 
            //climb

            break;
        default:
            //SPIN FOR APRILTAG
      }
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {

      switch (targetID) {
        case 3:
            //CODE FOR NON-CENTRAL RED SPEAKER
            //pivot to find the central apriltag
            //(or maybe just shoot using that one)
            
            break;
        case 4:
            //CODE FOR CENTRAL RED SPEAKER
            //face the robot towards speaker
            //calculate distance
            //move arm to right position
            //shoot

            break;
        case 5:
            //CODE FOR RED AMP
            //pivot arm up to set position for seeing apriltag
            //center ourselves on the apriltag
            //move forward slowly
            //put arm up at 90 degrees
            //shoot slowly

            break;
        case 9:
            //CODE FOR FAR HP-STATION 
            //face the apriltag
            //move to a close distance
            //center between the two apriltags

            break;
        case 10:
            //CODE FOR CLOSE HP-STATION
            //face the apriltag
            //move to a close distance
            //center between the two apriltags

            break;
        case 11:
            //RED STAGE
            //center yourself 
            //climb

            break;
        case 12:
            //RED STAGE
            //center yourself 
            //climb

            break;
        case 13:
            //RED STAGE
            //center yourself 
            //climb

            break;
        default:
            //SPIN FOR APRILTAG
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
