package frc.robot.vision.commands;

import org.opencv.core.Point;
import frc.robot.vision.Vision;

public class AutoTurretShoot {
    

    Point detectTarget() {
        //CHECK IF THE LIMELIGHT HAS A TARGET
        if (Vision.a_tV == true ) {
            // Get target coordinates (x, y) in screen space
            double targetX = Vision.a_tX;
            double targetY = Vision.a_tY;

            // Return target coordinates
            return new Point(targetX, targetY);
        } else {
            // No target detected, return null
            return null;
        }
    }
    
}
