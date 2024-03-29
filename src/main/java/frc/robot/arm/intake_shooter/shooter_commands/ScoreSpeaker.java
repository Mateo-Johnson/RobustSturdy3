// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.shooter_commands;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Constants.DriveConstants;


public class ScoreSpeaker extends Command {


    public static final CANSparkMax intake1 = DriveConstants.rightIntake;
    public static final CANSparkMax intake2 = DriveConstants.leftIntake;
    
    public static CANSparkMax rightOuttake = new CANSparkMax(21, MotorType.kBrushless);  
    public static CANSparkMax leftOuttake = new CANSparkMax(22, MotorType.kBrushless); 
    public static RelativeEncoder encoder = leftOuttake.getEncoder();


  /** Creates a new silly. */
  public ScoreSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double LLLLL = encoder.getVelocity();
    SmartDashboard.putNumber("silly11111", LLLLL);

    if (LLLLL >= 0.4) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
      rightOuttake.set(-5); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING, RIGHT RUNS TOP, LEFT RUNS BOTTOM
      leftOuttake.set(3.76665); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING

      intake1.set(-0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
      intake2.set(-0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
      //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT
 
    } else if (LLLLL <= 0.4) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
      rightOuttake.set(-5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      leftOuttake.set(3.76665); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rightOuttake.set(0);
    leftOuttake.set(0);
    intake1.set(0);
    intake2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}