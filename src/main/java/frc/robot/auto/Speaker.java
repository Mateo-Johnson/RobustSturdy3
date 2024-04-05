// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.arm.Arm;
import frc.robot.arm.intake_shooter.Intake_shooter;
import frc.robot.arm.intake_shooter.shooter_commands.ScoreSpeaker;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.utils.Constants.ControllerConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.vision.Vision;

public class Speaker extends Command {
  /** Creates a new Speaker. */

    public static final CANSparkMax intake1 = DriveConstants.rightIntake;
    public static final CANSparkMax intake2 = DriveConstants.leftIntake;
    public static CANSparkMax rightOuttake = ScoreSpeaker.rightOuttake;
    public static CANSparkMax leftOuttake = ScoreSpeaker.leftOuttake;
    public static RelativeEncoder encoder = leftOuttake.getEncoder();

    PIDController armPID = Arm.armPID;
    PIDController turningPID = new PIDController(0.0159, 0.00, 0.004);

    private final DriveSubsystem driveSubsystem;
    double heading;

    double tY;
    double tX;
    int targetID;
    double deadzone = 3;
    double move;

  public Speaker(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = encoder.getVelocity();
    SmartDashboard.putNumber("Velocity", velocity);

    if (Vision.a_tV = true) {

      tY = Vision.a_tY; //THE Y OFFSET OF THE TARGET
      tX = Vision.a_tX; //THE Y OFFSET OF THE TARGET
      targetID = (int) Vision.a_targetID; //SET TARGET ID EQUAL TO THE INT VALUE FROM VISION.JAVA BUT CAST IT TO INT

      double turnValue = turningPID.calculate(tX, 0);
      double distance = Vision.calculateDistance(tY, Arm.degrees, targetID, "speaker");
      double angle = Intake_shooter.calculateAngle(distance);
      move = armPID.calculate(Arm.degrees, angle);

      driveSubsystem.drive(
          -MathUtil.applyDeadband(RobotContainer.primaryDriver.getLeftY(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT X SPEED
          -MathUtil.applyDeadband(RobotContainer.primaryDriver.getLeftX(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT Y SPEED
          turnValue, //CONTROL THE ROBOT ROTATION
          false, true);

      if (tX > deadzone && tX < -deadzone) {

        Arm.rotateVector(-move);

        if (Arm.degrees > angle+deadzone && Arm.degrees < angle-deadzone) {

          if (velocity >= 0.4) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
            rightOuttake.set(-5); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING, RIGHT RUNS TOP, LEFT RUNS BOTTOM
            leftOuttake.set(3.76665); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING
      
            intake1.set(-0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
            intake2.set(-0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
            //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT
      
          } else if (velocity <= 0.4) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
            rightOuttake.set(-5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
            leftOuttake.set(3.76665); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
            //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
            }
        }
      }
    } else {

      move = armPID.calculate(Arm.degrees, 45);
      Arm.rotateVector(-move);

    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.rotateVector(0);
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
