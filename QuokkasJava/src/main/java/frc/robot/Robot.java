// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.PS5ControllerPorts;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot { 

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_IDchooser = new SendableChooser<>();
  private final String kAutoNameDefault = "Default";
  private String m_autoSelected;
  private String m_TargetID;
  private PS5Controller m_manipController;
  private PS5Controller m_driveController;

  PhotonCamera camera;
  PhotonTrackedTarget target;
  /* Mechanisms */
  private Drive drive;
  private Manipulator manipulator; // Change variable name to match your class name

  private double curr_arm_target;

  private Timer autotime = PhotonVisionConstants.autotime;

  String dyslexic = "leftMultinote";
  String dyslexic1  ="rightMultinote";

  // PID constants should be tuned per robot
  PIDController turnController =
      new PIDController(PhotonVisionConstants.ANGULAR_P, 0, PhotonVisionConstants.ANGULAR_D);

  @Override
  public void robotInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {

     dyslexic1 = "leftMultinote";
     dyslexic  ="rightMultinote";
      
      
     
    }
    if (ally.get() == Alliance.Blue) {

     dyslexic = "leftMultinote";
     dyslexic1 ="rightMultinote";
      
      
      
    }
    }
    drive = Drive.getInstance();
    manipulator = Manipulator.getInstance();
    curr_arm_target = Manipulator.kARM_START_POS; 
    // (manipulator.getArmEnc()) put this code for NOT going into the desired position

    m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.addOption("Basic", "Basic");
    m_chooser.addOption("MultiNote", "MultiNote");
    m_chooser.addOption("leftMultiNote", "leftMultiNote");
    m_chooser.addOption("SendIt", "SendIt");
    m_chooser.addOption(dyslexic, dyslexic);
    m_chooser.addOption(dyslexic1, dyslexic);

    SmartDashboard.putData("Auto Modes", m_chooser);

    m_driveController = new PS5Controller(PS5ControllerPorts.DRIVETRAIN_PORT);
    m_manipController = new PS5Controller(PS5ControllerPorts.MANIPULATOR_PORT);

    camera = new PhotonCamera("Camera");
    manipulator.resetShooters();
    drive.zeroGyro();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm enc", manipulator.getArmEnc());
    SmartDashboard.putNumber("D-Sensor Range", manipulator.getRange());
    SmartDashboard.putNumber("Shoot VelocityA", manipulator.getShooterAVelocity());
    SmartDashboard.putNumber("Shoot VelocityB", manipulator.getShooterBVelocity());
    SmartDashboard.putNumber("Gyro Angle", drive.getGyroAngle());
    SmartDashboard.putNumber("Encoders", drive.getEncodersPosition());
    SmartDashboard.putNumber("shooterA Encoder pos", manipulator.getaEncoderPosition());
    SmartDashboard.putNumber("shooterBEncoder pos", manipulator.getbEncoderPosition());

    
  }

  @Override
  public void autonomousInit() {
    autotime.start();

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    
    drive.zeroGyro();
    drive.resetEncoders();
    manipulator.resetShooters();
    autotime.reset();
    
  }

  @Override
  public void autonomousPeriodic() {
    
    int ApriltagID = 0;
    int direction = 0;

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
      
      ApriltagID = 3;

      direction = -1;
      
    }
    if (ally.get() == Alliance.Blue) {

      ApriltagID = 8;

      direction = 1; 
      
    }
    }
    else {
      ApriltagID = 8;
    
      }
      double pos = drive.getEncodersPosition();
       SmartDashboard.putNumber("pos", pos);
    
       /* BASIC */
    if ("Basic".equals(m_autoSelected)) {

      if (autotime.get() < 2.5) {
        manipulator.shoot(1);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      } else if (autotime.get() < 3) {
        manipulator.intake(1);
        manipulator.shoot(0.5);
      } else if (autotime.get() < 5) {
        drive.gyroDrive(0.3, 0.0);
      } else {
        drive.gyroDrive(0.0, 0.0);
      }

      /* MULTINOTE */
    } else if ("MultiNote".equals(m_autoSelected)) {

      if (autotime.get() < 2.0) {
        // Lower arm
        manipulator.shoot(1);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      } else if (autotime.get() < 4.0) {
        // Shoot
        manipulator.intake(1.0);
        manipulator.shoot(0.5);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      } else if (autotime.get() < 5.5) {
        // Drive, intake
        if (manipulator.getNoteSensor()) {
          manipulator.intake(0.375);
        } else {
          manipulator.intake(0.0);
        }
        manipulator.shoot(0.0);
        manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
        if (pos >= 1.3) {
        drive.gyroDrive(0.0, 0);
      } else {
         drive.gyroDrive(0.375, 0);
       }

      }  else if (autotime.get() < 7.0) {
        // Drive back
        manipulator.intake(0.0);
        manipulator.shoot(0.5);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
        drive.gyroDrive(-0.3, 0.0);
      } else if (autotime.get() < 12) {
        // Drive back
        manipulator.intake(1.0);
        manipulator.shoot(0.5);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
        drive.gyroDrive(0.0, 0.0);
      } else {
        // Finally,
        drive.move(0.0, 0.0); // Stop
        manipulator.intake(0.0);
        manipulator.shoot(0.0);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      }
    
      /* SEND IT */
    } else if ("SendIt".equals(m_autoSelected)) {

       if (autotime.get() < 2.0) {
        // Lower arm
        manipulator.shoot(1);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      } else if (autotime.get() < 4.0) {
        // Shoot
        manipulator.intake(1.0);
        manipulator.shoot(0.5);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      } else if (autotime.get() < 5.5) {
        // Drive, intake
        if (manipulator.getNoteSensor()) {
          manipulator.intake(0.375);
        } else {
          manipulator.intake(0.0);
        }
        manipulator.shoot(0.0);
        manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
        if (pos >= 1.5) {
        drive.gyroDrive(0.0, 0);
      } else {
         drive.gyroDrive(0.375, 0);
       }
      } else if (autotime.get() < 7.0) {
        manipulator.intake(0.0);
        manipulator.shoot(1.0);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        manipulator.armToPos(shot_angle);

        var result = camera.getLatestResult();
        double goalTarget = 0;

        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
            {
              goalTarget = targets.get(i).getYaw();
            }
          }
          drive.move(0, -turnController.calculate(goalTarget, 0));
        }
      } else if (autotime.get() < 9.0) {
        manipulator.intake(1.0);
        manipulator.shoot(0.5);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        manipulator.armToPos(shot_angle);

        var result = camera.getLatestResult();
        double goalTarget = 0;

        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
            {
              goalTarget = targets.get(i).getYaw();
            }
          }
          drive.move(0, -turnController.calculate(goalTarget, 0));
        }
      } else if (autotime.get() < 11.0) {
        if (manipulator.getNoteSensor()) {
          manipulator.intake(0.375);
        } else {
          manipulator.intake(0.0);
        }
        manipulator.shoot(0.0);
        manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
        drive.gyroDrive(0.2, -90.0*direction);
      } else if (autotime.get() < 12.0) {
        manipulator.intake(0.0);
        manipulator.shoot(0.3);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
        drive.gyroDrive(0.0, -37.0*direction);
      } else if (autotime.get() < 13.5) {
        manipulator.intake(0.0);
        manipulator.shoot(1.0);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        manipulator.armToPos(shot_angle);

        var result = camera.getLatestResult();
        double goalTarget = 0;

        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
            {
              goalTarget = targets.get(i).getYaw();
            }
          }
          drive.move(0, -turnController.calculate(goalTarget, 0));
        }
      } else if (autotime.get() < 14.5) {
        manipulator.intake(1.0);
        manipulator.shoot(0.5);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        manipulator.armToPos(shot_angle);

        var result = camera.getLatestResult();
        double goalTarget = 0;

        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
            {
              goalTarget = targets.get(i).getYaw();
            }
          }
          drive.move(0, -turnController.calculate(goalTarget, 0));
        }
      } else {
        drive.move(0.0, 0.0);
        manipulator.intake(0.0);
        manipulator.shoot(0.0);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      }

      /* LEFT MULTINOTE */
      /* ES LEFT DESDE EL PUNTO DE VISTA DEL BLUE ALLIANCE DRIVE STATION HACIA ENFRENTE */
      } else if ("leftMultiNote".equals(m_autoSelected)) {

        if (autotime.get() < 1.5) {
        manipulator.shoot(1);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      } else if (autotime.get() < 3){
         manipulator.intake(1.0);
        manipulator.shoot(0.5);
        manipulator.armToPos(Manipulator.kARM_FENDER_POS);
         if (pos >= 0.4) {
        drive.gyroDrive(0.0, 0);
      } else {
         drive.gyroDrive(0.3, 0);
       } 
      } else if (autotime.get() < 4.9) {
        if (manipulator.getNoteSensor()) {
          manipulator.intake(0.375);
        } else {
          manipulator.intake(0.0);
        }
        manipulator.shoot(0.0);
        manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
        drive.gyroDrive(0.2, 62*direction);
      } else if (autotime.get() < 7.0) {
        manipulator.intake(0.0);
        drive.gyroDrive(0.0, 30*direction);
      } else if  (autotime.get() < 10.0) {

        manipulator.intake(0.0);
        manipulator.shoot(1.0);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        manipulator.armToPos(shot_angle);

        var result = camera.getLatestResult();
        double goalTarget = 0;

        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
            {
              goalTarget = targets.get(i).getYaw();
            }
          }
          drive.move(0, -turnController.calculate(goalTarget, 0));
        }
      } else if (autotime.get() < 13.5) {
        manipulator.intake(1.0);
        manipulator.shoot(0.5);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        manipulator.armToPos(shot_angle);
        // manipulator.armToPos(0.024);

        var result = camera.getLatestResult();
        double goalTarget = 0;

        if (result.hasTargets()) {
          List<PhotonTrackedTarget> targets = result.getTargets();

          for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
            {
              goalTarget = targets.get(i).getYaw();
            }
          }
          drive.move(0, -turnController.calculate(goalTarget, 0));
       } else if (autotime.get() < 15) {
        manipulator.intake(0.0);
        manipulator.shoot(0.0); 
       }
   } 
   /* RIGHT MULTINOTE */
   /* ES RIGHT DESDE EL PUNTO DE VISTA DEL BLUE ALLIANCE DRIVE STATION HACIA ENFRENTE */
  } else if ("rightMultiNote".equals(m_autoSelected)) {

    if (autotime.get() < 1.5) {
    manipulator.shoot(1);
    manipulator.armToPos(Manipulator.kARM_FENDER_POS);
  } else if (autotime.get() < 3){
     manipulator.intake(1.0);
    manipulator.shoot(0.5);
    manipulator.armToPos(Manipulator.kARM_FENDER_POS);
     if (pos >= 0.4) {
    drive.gyroDrive(0.0, 0);
  } else {
     drive.gyroDrive(0.3, 0);
   } 
  } else if (autotime.get() < 5.0) {
    if (manipulator.getNoteSensor()) {
      manipulator.intake(0.3);
    } else {
      manipulator.intake(0.0);
    }
    manipulator.shoot(0.0);
    manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
    drive.gyroDrive(0.2, -62*direction);
  } else if (autotime.get() < 7.0) {
    manipulator.intake(0.0);
    drive.gyroDrive(0.0, -30*direction);
  } else if  (autotime.get() < 10.0) {

    manipulator.intake(0.0);
    manipulator.shoot(1.0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
    double ty = table.getEntry("targetPixelsY").getDouble(0.0);
    double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
    manipulator.armToPos(shot_angle);

    var result = camera.getLatestResult();
    double goalTarget = 0;

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();

      for (int i = 0; i < targets.size(); i++) {
        if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
        {
          goalTarget = targets.get(i).getYaw();
        }
      }
      drive.move(0, -turnController.calculate(goalTarget, 0));
    }
  } else if (autotime.get() < 13.5) {
    manipulator.intake(1.0);
    manipulator.shoot(0.5);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
    double ty = table.getEntry("targetPixelsY").getDouble(0.0);
    double shot_angle =-0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
    manipulator.armToPos(shot_angle);
    // manipulator.armToPos(0.024);

    var result = camera.getLatestResult();
    double goalTarget = 0;

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();

      for (int i = 0; i < targets.size(); i++) {
        if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
        {
          goalTarget = targets.get(i).getYaw();
        }
      }
      drive.move(0, -turnController.calculate(goalTarget, 0));
   } 
  } else if (autotime.get() < 15) {
    manipulator.intake(0.0);
    manipulator.shoot(0.0); 
  } 
  } else {
  drive.gyroDrive(0.2, 0.0);
 }
} 
 

  @Override
  public void teleopInit() {

    m_TargetID = m_IDchooser.getSelected();
    System.out.println("Target ID: " + m_TargetID);

    drive.resetEncoders();
    manipulator.resetShooters();
    drive.zeroGyro();
  }

  @Override
  public void teleopPeriodic() {
   
    
     int ApriltagID = 0;

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {

      ApriltagID = 5;
       
    }
    if (ally.get() == Alliance.Blue) {

      ApriltagID = 8;
        
    }
    }
    else {
      ApriltagID = 8;
    
      }

   
    // Drive
    double power = 0;
    double steering = 0;

    if (m_driveController.getR2Axis() > 0.15) {
     // manipulator.shootVoltage(400);

    } else {
      //manipulator.shootVoltage(0);
    }

    if (m_driveController.getL2Axis() > 0.1) {
      power = m_driveController.getLeftY() * 0.5;
      steering = m_driveController.getRightX() * 0.5;

    } else {
      power = m_driveController.getLeftY() * 1;
      steering = m_driveController.getRightX() * 1;
    }

    if (Math.abs(power) < 0.15) {
      power = 0;
    }

    if (Math.abs(steering) < 0.15) {
      steering = 0;
    }

    drive.move(power, steering);

    /* Intake */
    if (m_manipController.getR1Button() && manipulator.getNoteSensor()) {
      // If pressing intake button, and the NOTE is not int the intake
      manipulator.intake(0.375);
      if (m_manipController.getR2Axis() < 0.5) {
        curr_arm_target = Manipulator.kARM_FLOOR_POS;
      }
    } else if (m_manipController.getL1Button()) {
      // Outtake
      manipulator.intake(-1.0);
      manipulator.shoot(-0.25);
    } else {
      // do nothing
      manipulator.intake(0.0);
      manipulator.shoot(0.0);
    }

    if (m_manipController.getR1Button() && manipulator.getNoteSensor()) {
      // If pressing the intake and NOTE is in the intake
      m_manipController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    } else {
      m_manipController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    if (m_manipController.getR1ButtonReleased()) {
      // No longer intaking, raise intake to avaoid damage
      curr_arm_target = Manipulator.kARM_FENDER_POS;
    }


    SmartDashboard.putNumber("ApriltagID", ApriltagID);
     


    if (m_driveController.getSquareButton()){

    var result = camera.getLatestResult();
      // Put the ID you want to follow or prioritize
      double goalTarget = 0;

      

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
      
        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
          {
            goalTarget = targets.get(i).getYaw();
          }
        }

        double Kp = 0.00567556; //0.00567556 Valores dados por Dios, gracias Dios :)
        double error = goalTarget;
        steering = Kp * error;

        //steering = -turnController.calculate(goalTarget, 0);
        // double cheap = steering;
       drive.move(0, steering);
        SmartDashboard.putNumber("moveAngle", steering);
        SmartDashboard.putNumber("GoalTarget", goalTarget);
      }
    }

    

    /* Shooter */
    if (m_manipController.getR2Axis() > 0.1) {
      if (manipulator.getArmEnc() > Manipulator.kARM_START_POS) {
        // if arm turned back farther than starting config
        manipulator.shoot(0.25);
      } else {

      var result = camera.getLatestResult();
      // Put the ID you want to follow or prioritize
      

      

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
      
        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == ApriltagID){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        // read values periodically
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
        double shotAngle =  -0.0000002832496 * Math.pow(ty, 2) + 0.0003691787531 * ty + 0.0175008871005;
        SmartDashboard.putNumber("shotAngle", shotAngle);
         double FarAngle =  -0.0000028840396 * Math.pow(ty, 2) + 0.0027452837529 * ty - 0.5193911599854;

        // double FarAngle = 0; // Function with far shots
        SmartDashboard.putNumber("PhotonY", ty);

        if (ty < 10){
          SmartDashboard.putNumber("off", 1);

        } else {
          SmartDashboard.putNumber("off", 2);
           if (ty > 420) {
       curr_arm_target = FarAngle;
       SmartDashboard.putNumber("FarAngle", FarAngle);
        } else {
        curr_arm_target = shotAngle;
        }

            
          }
        }

        
        }

      }
        
       
      }
    }

    if (m_manipController.getL2Axis() > 0.1) {
      manipulator.intake(1.0);
    }

    if (m_manipController.getR2Axis() > 0.5) {
      // if arm turned back farther than starting config, score AMP
      if (manipulator.getArmEnc() > Manipulator.kARM_START_POS) {
        manipulator.intake(1.0);
        manipulator.shoot(0.5);
      } else {
        // High goal shooting
        manipulator.shoot((m_manipController.getR2Axis() - 0.5) * 2);
      }

      if (m_manipController.getR1Button()) {
        // Run intake despite NOTE being in intake
        manipulator.intake(1.0);
      }
    } else {
      manipulator.shoot(0.1);
    }

    /*Arm manual control*/
    if (m_manipController.getTriangleButtonPressed()) {
      // Amp stating config
      curr_arm_target = Manipulator.kARM_AMP_POS;
    }

    if (m_manipController.getPOV(0) == 0) {
      manipulator.moveArm(-0.15); // Up
      curr_arm_target = manipulator.getArmEnc();
    } else if (m_manipController.getPOV(0) == 180) {
      manipulator.moveArm(0.15); // Down
      curr_arm_target = manipulator.getArmEnc();
    } else {
      manipulator.armToPos(curr_arm_target);
    }
    SmartDashboard.putNumber("Arm Target", curr_arm_target);
  }
}
