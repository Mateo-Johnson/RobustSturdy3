// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.intake_shooter.Intake_shooter;
import frc.robot.arm.intake_shooter.shooter_commands.ScoreSpeaker;
import frc.robot.drivetrain.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class IntakeRing extends Command {
  /** Creates a new silly. */
  private final ColorSensorV3 colorSensor = DriveSubsystem.colorSensor;
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color BlueTarget = new Color(0.143, 0.427, 0.429); //CREATES A COLOR
  private final Color GreenTarget = new Color(0.197, 0.561, 0.240); //CREATES A COLOR
  private final Color RedTarget = new Color(0.561, 0.232, 0.114); //CREATES A COLOR
  private final Color YellowTarget = new Color(0.361, 0.524, 0.113); //CREATES A COLOR
  private final Color OrangeTarget = new Color(0.453, 0.405, 0.141); //CREATES A COLOR
  public boolean hasRing;

  public IntakeRing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorMatcher.addColorMatch(BlueTarget);
    colorMatcher.addColorMatch(GreenTarget);
    colorMatcher.addColorMatch(RedTarget);
    colorMatcher.addColorMatch(YellowTarget); 
    colorMatcher.addColorMatch(OrangeTarget);
    hasRing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Color detectedColor = colorSensor.getColor();
    // String colorString;
    // ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    // if (match.color == BlueTarget) {
    //   colorString = "blue";
    //   ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE

    // } else if (match.color == RedTarget) {
    //   colorString = "red";
    //   Intake_shooter.stopWheels();
    //   ringInIntake(true); //FLAG TO MARK WHEN THERE IS A RING IN INTAKE - 

    // } else if (match.color == GreenTarget) {
    //   colorString = "green";
    //   ringInIntake (false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE

    // } else if (match.color == YellowTarget) {
    //   colorString = "yellow";
    //   Intake_shooter.runIntake(0.4);
    //   ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE 

    // } else if (match.color == OrangeTarget){
    //   colorString = "orange";
    //   Intake_shooter.stopWheels();
    //   ringInIntake(true); //FLAG TO MARK WHEN THERE IS A RING IN INTAKE - 

    // } else {
    //   colorString = "we can't tell";
    //   Intake_shooter.runIntake(0.4);
    //   ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE
    // }

    // SmartDashboard.putNumber("red value", detectedColor.red);
    // SmartDashboard.putNumber("green value", detectedColor.green);
    // SmartDashboard.putNumber("blue value", detectedColor.blue);
    // SmartDashboard.putNumber("confidence", match.confidence);
    // SmartDashboard.putString("detected color", colorString);
    ScoreSpeaker.intake1.set(-0.5);
    ScoreSpeaker.intake2.set(-0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Intake_shooter.stopWheels();
    // hasRing = false;
    ScoreSpeaker.intake1.set(0);
    ScoreSpeaker.intake2.set(0);

  }


  public void ringInIntake(boolean trueslashfalse) {
    hasRing = trueslashfalse;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasRing;
  }
} 