package frc.robot.arm.intake_shooter;

import edu.wpi.first.math.MathUtil;

public class Intake_shooter {

  public static double x;

  //SPEAKER SHOOTING FUNCTION THE REST OF THIS TEXT IS NOT NECESSARY I AM JUST USING THIS AS A LINE BREAK.

    public static double calculateAngle(double distance) {
    //NEEDS TO BE ROBOT-SPEAKER DISTANCE
    double distanceToTarget = distance;

    //CALCULATE DISTANCE (SETPOINT BASED)
    //MY DOUBLE ARRAY ARRAY
    double[][] aimingPoses = { //OK SO THESE ANGLES ARE ABOUT 3.5 DEGREES BELOW WHAT THEY SHOULD BE
        // new double[] {4, 20}, //20+3.5
        // new double[] {5, 28}, //28+3.5
        // new double[] {7, 34.7}, //24.7+3.5
        // new double[] {9, 37.4}, //37.4+3.5
        new double[] {4, x},
        new double[] {5, x},
        new double[] {6, x},
        new double[] {7, x},
        new double[] {8, x},
        new double[] {9, x}

    };

    //FIND THE UPPER AND LOWER BOUNDS OF THE AIMING POSES ARRAY USING THE DISTANCE TO TARGET  
    double[] upperBound = aimingPoses[aimingPoses.length - 1];
    double[] lowerBound = aimingPoses[0];

    for (int i = 0; i < aimingPoses.length; i++) {
      if (distanceToTarget > aimingPoses[i][0]) {
        lowerBound = aimingPoses[i];
      }
      if (distanceToTarget < aimingPoses[i][0]) {
        upperBound = aimingPoses[i];
        break;
      }
    }

    //CLAMPS IT BETWEEN 0 AND 1 AS A PERCENT
    double t = (distanceToTarget - lowerBound[0]) / (upperBound[0] - lowerBound[0]);

    return MathUtil.clamp(MathUtil.interpolate(lowerBound[1], upperBound[1], t), 0, 90);
  }
}