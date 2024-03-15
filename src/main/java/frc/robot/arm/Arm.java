package frc.robot.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.Constants.DriveConstants;


public class Arm {

    public static CANSparkMax leftArm = DriveConstants.leftArm;  
    public static CANSparkMax rightArm = DriveConstants.rightArm;
    public static AbsoluteEncoder armEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);
    public static double armEncoderReading = armEncoder.getPosition();
    private static final int cyclesPerRotation = 2048;
    private static final int drivenGearTeeth = 60;
    private static final int driveGearTeeth = 15;

    public static double gearRatio = (double) drivenGearTeeth / driveGearTeeth;
    public static int encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio);
    public static double degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution; // Corrected line
    public static double degrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle;

    public static double aMP = 0.8;
    public static double aMI = 0.1;
    public static double aMD = 0.51;

    public static PIDController armPID = new PIDController(aMP, aMI, aMD);


    public static void rotateVector(double speed) {
      leftArm.set(speed);
      rightArm.set(-speed);
    }
    
}