package frc.robot.arm.intake_shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.DriveConstants;

public class Intake_shooter {
      //INTAKE
    public static final CANSparkMax intake1 = DriveConstants.rightIntake;
    public static final CANSparkMax intake2 = DriveConstants.leftIntake;
    public static final CANSparkMax rightOuttake = DriveConstants.rightOuttake;
    public static final CANSparkMax wrongOuttake = DriveConstants.leftOuttake;
    private static RelativeEncoder shooterEncoder = rightOuttake.getEncoder();

  
    public static void runIntake(double speed) {
        intake1.set(speed);
        intake2.set(-speed);
    }
    public static void stopWheels(){
        intake1.stopMotor();
        intake2.stopMotor();
        rightOuttake.stopMotor();
        wrongOuttake.stopMotor();
      }
    public static void runOuttake() {
        SmartDashboard.putNumber("silly", shooterEncoder.getVelocity());
    if (shooterEncoder.getVelocity() >= 1) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
      rightOuttake.set(5); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING, RIGHT RUNS TOP, LEFT RUNS BOTTOM
      wrongOuttake.set(3.76665); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING

      rightOuttake.set(0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
      wrongOuttake.set(-0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
      //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT
 
    } else if (shooterEncoder.getVelocity() <= 1) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
      rightOuttake.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      wrongOuttake.set(3.76665); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
    }
  }
    public static void runIntakeAndOuttake(double speed) {
        intake1.set(speed);
        intake2.set(-speed);
        rightOuttake.set(speed);
        wrongOuttake.set(-speed);
      }
}