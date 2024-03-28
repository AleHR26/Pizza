package frc.robot.Util;

import java.util.List;
import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Mat.Atable;

import frc.robot.Util.Physics.ExtraMath;
import frc.robot.Util.Physics.RaycastSimulation;



public class AutoArm {
    
  public static class AprilTagSpecs{
    String name;
    doubleRange goalHeight; //doubleRange of entrance in cm
    double goalAngle; //entrance angle in cm
    boolean intakerFace;

    public AprilTagSpecs(String name, doubleRange goalHeight, double goalAngle, boolean intakerFace){
      this.name = name;
      this.goalHeight = goalHeight;
      this.goalAngle = goalAngle;
      this.intakerFace = intakerFace;
    }
  }
  public static Map<Integer, AprilTagSpecs> aprilTagsMap;


  public static class PresetPosition{
    String name;
    double armAngle;

    public PresetPosition(String name, double armAngle){
      this.name = name;
      this.armAngle = armAngle;
    }
  }
  public static List<PresetPosition> presetPositions;
  

  public static class doubleRange{
    double min;
    double max;

    public doubleRange(double min, double max){
      this.min = min;
      this.max = max;
    }
  }
  public static class pointedVector{
    double magnitude;
    double angle;

    public pointedVector(double magnitude, double angle){
      this.magnitude = magnitude;
      this.angle = angle;
    }
  }




  /**
     * Positive angles imply back-to-forward positioning of the robot's arm stem.
     */
  public static double Arm_Elevated_angle = 75.9;
  /**
     * Perpendicular to the final shooting angle, it is the facing direction in conjunction 
     * to the whole robot arm.
     */
  public static final double Angle_Difference_Arm_Stem_To_Shooter_Head_Frontal = 30.6;
  /**
     * (90°) Constant value that adds for perpendicularity on the final shooting angle.
     */
  public static final double Shooter_Muzzle_To_Shooter_Frontal_Perpendicularity = 90;
  /**
     * The distance in cm between the ground, from bottom wheels, to the center point of the arm gears.
     */
  public static final double Distance_Difference_Ground_To_Arm_Gear = 27.428;
  /**
     * The distance in cm between the furthermost part of the forward bumper to the center point 
     * of the arm gears.
     */
  public static final double Distance_Difference_Bumper_Frontal_To_Arm_Gear = 21.325;
  /**
     * Imaginary line, lengthed in cm between the center point of the arm gears to the exact mozzle exit point 
     * from the shooter component.
     */
  public static final double Imaginary_Third_Length = 59.487;
  /**
     * The angular difference between the arm stem to the exact mozzle exit point from the shooter 
     * component. 
     */
  public static final double Angle_Difference_Arm_Stem_To_Imaginary_Third = 16.9;
  /**
     * The angular difference between the imaginary third to the exact mozzle exit point from the shooter 
     * component. 
     */
  public static final double Angle_Difference_Imaginary_Third_To_Shooter_Muzzle = 76.3;
  /**
     * Minimum angle the arm may rest upon without hitting the ground.
     */
  public static final double Arm_Angle_Offset = 0.0; //63.216




  public static Point2D.Double GetShooterMuzzleDisplacement(double inputArmPosAngle)
  {
    double clampedInputArmPosAngle = ExtraMath.clamp(
      inputArmPosAngle, Arm_Angle_Offset, Arm_Angle_Offset+180);

    double counterArmPosAngleRadians = Math.toRadians(
      180 - (clampedInputArmPosAngle + Angle_Difference_Arm_Stem_To_Imaginary_Third));

    double disX = Imaginary_Third_Length * Math.cos(counterArmPosAngleRadians);
    double disY = Imaginary_Third_Length * Math.sin(counterArmPosAngleRadians);

    return new Point2D.Double(disX, disY + Distance_Difference_Ground_To_Arm_Gear);
  }

  public static double GetShooterMuzzleAimAngle(double inputArmPosAngle){
    return
        180-(inputArmPosAngle + Angle_Difference_Arm_Stem_To_Shooter_Head_Frontal + Shooter_Muzzle_To_Shooter_Frontal_Perpendicularity);
  }

  public static double GetShootAngleTowardsTarget(double xDistance, AprilTagSpecs ATS)
  {
    Point2D.Double targetMin = 
      new Point2D.Double(
        xDistance + Distance_Difference_Bumper_Frontal_To_Arm_Gear, 
        ATS.goalHeight.min);
    
    double angleRadians = Math.toRadians(ATS.goalAngle);
    Point2D.Double targetMax = 
    
      new Point2D.Double(
        targetMin.x + ATS.goalHeight.max * Math.cos(angleRadians),
        targetMin.y + ATS.goalHeight.max * Math.sin(angleRadians));

    Point2D.Double target = new Point2D.Double((targetMin.x + targetMax.x) / 2 , /*(targetMin.y + targetMax.y) / 2*/190 );
    
    double armGoalAngle = Physics.RaycastSimulation.shootRaycastToGoalArmAngle(target);
    
    // Print output for variables
    System.out.println("targetMin: " + targetMin.toString());
    System.out.println("targetMax: " + targetMax.toString());
    System.out.println("target: " + target.toString());

    return armGoalAngle;
  }

  
  // public static final AprilTagSpecs Stage_ATS = new AprilTagSpecs(  **LAS MEDIDAS SON ERRONEAS**
  //   "Stage", new doubleRange(93.2434, 15.3416), 90, true);

  public static final AprilTagSpecs AMP_ATS = new AprilTagSpecs(
    "AMP", new doubleRange(66.04, 8.9), 0, false); //horizontal goal

  public static final AprilTagSpecs Speaker_ATS = new AprilTagSpecs(
    "Speaker", new doubleRange(198.45, 47.569), 165.244, false);

  public static void Init()
  {
    aprilTagsMap = new HashMap<Integer, AprilTagSpecs>();

    aprilTagsMap.put(5, AMP_ATS);
    aprilTagsMap.put(6, AMP_ATS);

    aprilTagsMap.put(3, Speaker_ATS);
    aprilTagsMap.put(4, Speaker_ATS);

    presetPositions = new ArrayList<>();
    presetPositions.add(null);

  }





  // TO ACTIONATE ON ROBOT
  /**
     * Sets the Arm position by a given angle.
     - REMINDER: The angle is added from behind the shooter component and forwards. It must be > 0.1756.
     */
  public static void SetArmPositionAngle(double inputArmPosAngle)
  {

  }


}
