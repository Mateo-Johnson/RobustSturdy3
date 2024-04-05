package frc.robot.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;


public class Arm extends SubsystemBase{

  public static CANSparkMax leftArm = DriveConstants.leftArm;  
  public static CANSparkMax rightArm = DriveConstants.rightArm;
  public static AbsoluteEncoder armEncoder = leftArm.getAbsoluteEncoder(Type.kDutyCycle);
  public static final int cyclesPerRotation = 2048;
  public static final int drivenGearTeeth = 60;
  public static final int driveGearTeeth = 15;
  public static double degrees;

  @Override
  public void periodic() {
    double armEncoderReading = armEncoder.getPosition();
    double gearRatio = (double) drivenGearTeeth / driveGearTeeth;
    int encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio);
    double degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution;

    if (degrees <= 0) {
      degrees = 1;
    } else {
      degrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle;
    }

    SmartDashboard.putNumber("Arm Angle", degrees);
  }

    public static PIDController armPID = new PIDController(0.015, 0, 0);

    public static void rotateVector(double speed) {
      leftArm.set(speed);
      rightArm.set(-speed);
    }
    
}