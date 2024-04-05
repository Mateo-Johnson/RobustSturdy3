package frc.robot.auto.scoring;

import edu.wpi.first.math.MathUtil;

//THE STARTING POSITIONS FOR AUTO: 
//TOP / LEFT POSITION - 1.9/7.0
//MIDDLE POSITION - 1.9/5.5
//BOTTOM / RIGHT - 1.9/4.1
//BASICALLY, ALIGN IT TO THE RING SPOT


public class Auto {
      public static double calculateAngle(double distance) {
    // diffPose.getX() needs to be the distance from the robot to the speaker
    double distanceToTarget = distance;

    //CALCULATE DISTANCE (SETPOINT BASED)
    //MY DOUBLE ARRAY ARRAY
    double[][] aimingPoses = { //FIND NEW ONES THESE ARE PLACEHOLDERS
        new double[] {4, 20}, 
        new double[] {5, 28}, 
        new double[] {7, 34.7}, 
        new double[] {9, 37.4}, 
        new double[] {1.485, 60.0}, 
        new double[] {1.485, 60.0}, 
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
