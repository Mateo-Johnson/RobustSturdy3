package frc.robot.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.robot.utils.Constants.DriveConstants;


public class Arm {

    public static CANSparkMax leftArm = DriveConstants.leftArm;  
    public static CANSparkMax rightArm = DriveConstants.rightArm;
    public static AbsoluteEncoder armEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);
    public static double armEncoderReading = armEncoder.getPosition();


    public static void rotateVector(double speed) {
      leftArm.set(speed);
      rightArm.set(-speed);
    }
    
}
