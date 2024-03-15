// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.shooter_commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.arm.Arm;
import frc.robot.arm.intake_shooter.Intake_shooter;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.vision.Vision;

public class ScoreSpeaker extends Command {


    public static final CANSparkMax intake1 = DriveConstants.rightIntake;
    public static final CANSparkMax intake2 = DriveConstants.leftIntake;
    public static final CANSparkMax rightOuttake = DriveConstants.rightOuttake;
    public static final CANSparkMax wrongOuttake = DriveConstants.leftOuttake;
    private static RelativeEncoder shooterEncoder = rightOuttake.getEncoder();
    double dist;
    double power;

    CommandXboxController controller =  RobotContainer.primaryDriver;
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
    SmartDashboard.putNumber("silly", shooterEncoder.getVelocity());

    if (Vision.a_tV) {
      if (shooterEncoder.getVelocity() >= calculateVelocity(power)) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
        rightOuttake.set(power); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING, RIGHT RUNS TOP, LEFT RUNS BOTTOM
        wrongOuttake.set(power); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING
  
        intake1.set(0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
        intake2.set(-0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
        //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.5); //RUMBLE THE CONTROLLER WHEN ITS AIMED
         
      } else if (shooterEncoder.getVelocity() <= calculateVelocity(power)) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
        rightOuttake.set(power); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
        wrongOuttake.set(power); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
        //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
      }
    } else {
      if (shooterEncoder.getVelocity() >= 17) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
        rightOuttake.set(5); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING, RIGHT RUNS TOP, LEFT RUNS BOTTOM
        wrongOuttake.set(5); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING
  
        intake1.set(0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
        intake2.set(-0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
        //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.5); //RUMBLE THE CONTROLLER WHEN ITS AIMED
         
      } else if (shooterEncoder.getVelocity() <= 17) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
        rightOuttake.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
        wrongOuttake.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
        //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
      }
    }

    dist = Vision.calculateDistance(Vision.a_tY, Arm.degrees, Vision.a_targetID, "speaker");
    power = calculatePower(dist);
  }

    //IF WE KNOW THE BEST ANGLE FOR WHEN WE'RE CLOSE AND THE BEST FOR WHEN WE'RE FAR AWAY, THEN WE KNOW THE IN-BETWEEN
    public static double calculatePower(double distance) {
      //CONSTANTS FOR THE TWO POINTS
      double dist1 = 0.00; //DISTANCE = 0 FEET PLACEHOLDER
      double power1 = 0.00; //ARM POWER = 0 POWER PLACEHOLDER
    
      double dist2 = 10.00; //DISTANCE = 10 FEET PLACEHOLDER 
      double power2 = 1.00; //ARM POWER = 1 POWER PLACEHOLDER
    
      //LINEAR INTERPOLATION FORMULA 
      double armAngle = power1 + ((distance - dist1) / (dist2 - dist1)) * (power2 - power1);
    
      return armAngle;
    }

    public static double calculateVelocity(double power) {
      final double maxRPM = 2428.57; //MAX RPM OF MOTOR
      final double setpoint = power; //SETPOINT
      final double ticksPerRevolution = 42.0; //ENCODER TICKS PER MOTOR SHAFT REVOLUTION
      final double encoderTicksPerSecond = (maxRPM * ticksPerRevolution) / 60.0; //CALCULATE ENCODER TICKS PER SECOND
      final double encoderTicksPer100ms = encoderTicksPerSecond / 10.0; // CONVERT TO ENCODER TICKS PER 100 MILLIS

      //CALCULATE EQUIVALENT ENCODER TICKS PER 100 MILLIS
      double equivalentTicks = setpoint * encoderTicksPer100ms;

      return equivalentTicks;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake_shooter.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}