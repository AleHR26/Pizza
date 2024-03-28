package frc.robot.Util;

import java.awt.geom.Point2D;
import java.util.Scanner;

import frc.robot.Util.AutoArm.doubleRange;

public class Physics {

    public class ProjectileMotion {
        // Constants for NOTE properties
        // private static final double INSIDE_DIAMETER = 10.0 / 39.37; // Convert inches to meters
        // private static final double OUTSIDE_DIAMETER = 14.0 / 39.37; // Convert inches to meters
        // private static final double THICKNESS = 2.0 / 39.37; // Convert inches to meters
        private static final double NOTE_WEIGHT = (8.3) / 35.274; // Convert ounces to kilograms
    
        // Constants for physics calculations
        private static final double GRAVITY = 9.81; // Acceleration due to gravity in m/s^2
    
        /**
         * Calculates the projectile motion of a foam disk.
         * @param force The force applied to the foam disk in Newtons.
         * @param angle The launch angle of the foam disk in degrees.
         * @param altitude The altitude from which the foam disk is launched in meters.
         * @return An array containing the horizontal distance and maximum height reached by the foam disk.
         */
        public static double[] calculateProjectileMotion(double angle, double altitude) {
            // Convert launch angle from degrees to radians
            double angleRad = Math.toRadians(angle);

            // Calculate force applied to the foam disk
            double force = NOTE_WEIGHT * GRAVITY;

            // Calculate initial velocity components
            double initialVelocityX = force * Math.cos(angleRad);
            double initialVelocityY = force * Math.sin(angleRad);

            // Calculate time of flight
            double timeOfFlight = (2 * initialVelocityY) / GRAVITY;

            // Calculate horizontal distance traveled
            double horizontalDistance = initialVelocityX * timeOfFlight;

            // Calculate maximum height reached
            double maxHeight = (initialVelocityY * initialVelocityY) / (2 * GRAVITY) + altitude;

            return new double[]{horizontalDistance, maxHeight};
        }
    }


    // public static void main(String[] args) {
    //     Point2D.Double origin = new Point2D.Double(-2.0, 8.0); // Origin point
    //     Point2D.Double target = new Point2D.Double(10.0, 20.0); // Target point
    //     double hitAngle = RaycastSimulation.shootRaycastVsPoint(origin, target);
        
    //     if (hitAngle >= 0) {
    //         System.out.println("Raycast hit target at angle: " + hitAngle + " degrees");
    //     } else {
    //         System.out.println("Raycast did not hit the target.");
    //     }
    // }



    public class RaycastSimulation {

        public static double shootRaycastToGoalArmAngle(Point2D.Double target) 
        {
            double increment = 1e-1;
            double armAngle = AutoArm.Arm_Angle_Offset;
            
            System.out.println("Beginning operation!");
            long startTime = System.nanoTime();
    
            while (armAngle <= AutoArm.Arm_Angle_Offset + 180.0) {
                double muzzleAngle = AutoArm.GetShooterMuzzleAimAngle(armAngle);
                Point2D.Double origin = AutoArm.GetShooterMuzzleDisplacement(armAngle);

                // Check if the raycast intersects with the target within a small margin of error
                if (Geometry.doesIntersect(origin, muzzleAngle, target)) 
                {
                    {long endTime = System.nanoTime();
                    double elapsedTimeSeconds = (endTime - startTime) / 1e9;
                     System.out.println("//////////////////////////////////////  RAYCAST SUCCESSFULL ///////////////////////////////////////////");
                     System.out.println("Raycast at (origin) : "+origin.toString());
                     System.out.println("Angle of Shooter Muzzle: " + muzzleAngle + " degrees");
                     System.out.println("Goal Angle of Arm: " + armAngle + " degrees");
                    System.out.println("Elapsed time: " + elapsedTimeSeconds + " seconds");}

                    return armAngle; // Return the angle at which the raycast hits the target
                }
                
                // System.out.println("Raycast at: "+origin.toString()+", ArmAngle: "+armAngle+", MuzzleAngle: "+muzzleAngle+"} did not intersect point!");
                // Increment the angle for the next iteration
                armAngle += increment;
            }
            
            {long endTime = System.nanoTime();
            System.out.println("//////////////////////////////////////  RAYCAST FAILURE ////////////////////////////////////////////////");
            double elapsedTimeSeconds = (endTime - startTime) / 1e9;
            System.out.println("No proper Goal Angle of Arm rotation was found!");
            System.out.println("No proper angle of Shooter Muzzle rotation was found!");
            System.out.println("Elapsed time: " + elapsedTimeSeconds + " seconds");}

            return -1; // Indicates that the raycast did not hit the target within 360 degrees
        }

        public static double shootRaycastVsPoint(Point2D.Double origin, Point2D.Double target) 
        {
            double increment = 1e-3;
            double angle = 0.0;
            
            System.out.println("Beginning operation!");
            long startTime = System.nanoTime();
    
            while (angle <= 360.0) {
                // Check if the raycast intersects with the target within a small margin of error
                if (Geometry.doesIntersect(origin, angle, target)) 
                {
                    {long endTime = System.nanoTime();
                    double elapsedTimeSeconds = (endTime - startTime) / 1e9;
                    System.out.println("Angle of rotation: " + angle + " degrees");
                    System.out.println("Elapsed time: " + elapsedTimeSeconds + " seconds");}

                    return angle; // Return the angle at which the raycast hits the target
                }
                
                // System.out.println("Raycast at {"+origin.x+", "+origin.y+", Angle:"+angle+"} did not intersect point {"+target.x+", "+target.y+"}!");
                // Increment the angle for the next iteration
                angle += increment;
            }
            
            {long endTime = System.nanoTime();
            double elapsedTimeSeconds = (endTime - startTime) / 1e9;
            System.out.println("Angle of rotation: " + angle + " degrees");
            System.out.println("Elapsed time: " + elapsedTimeSeconds + " seconds");}

            return -1; // Indicates that the raycast did not hit the target within 360 degrees
        }
    }





    public class Geometry {

        // Function to check if the line intersects with a given point
        public static boolean doesIntersect(Point2D.Double origin, double angle, Point2D.Double target) {
            double epsilon = 1;
            double angleRadians = Math.toRadians(angle);

            // Calculate direction components
            double directionX = Math.cos(angleRadians);
            
            // Check if direction is zero (parallel to y-axis)
            if (directionX == 0) {
                return origin.x == target.x;
            }

            // Calculate parameter t
            double t = (target.x - origin.x) / directionX;

            // Check if point is on ray or line
            if (t < 0) {
                // Point is behind the origin
                return false;
            }

            // Calculate y-coordinate of the point on the line at parameter t
            double lineY = origin.y + t * Math.sin(angleRadians);

            // Check if the y-coordinate of the point on the line is the same as the point's y-coordinate
            return Math.abs(lineY - target.y) < epsilon; // You may need to adjust the epsilon value based on your precision requirements
        }

    }

    public class ExtraMath {

        // Clamp function equivalent to Unity's Math.Clamp
        public static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(value, max));
        }
    }




}
