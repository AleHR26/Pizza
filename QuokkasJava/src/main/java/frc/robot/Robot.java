// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PS5ControllerPorts;
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.MultiNote;
import frc.robot.autonomous.SendIt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot {

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String kAutoNameDefault = "Default";
  private final String kAutoNameCustom = "My Auto";
  private static final String Sendit = "Sendit/3notes";
  private static final String Multinote = "Multinote/2notes";
  private static final String Basic = "Basic/drive";
  private String m_autoSelected;
  private PS5Controller m_manipController;
  private PS5Controller m_driveController;
  private Timer autoTimer;
  private double lastTimestamp; // Add this line for missing variable

  PhotonCamera camera;
  PhotonTrackedTarget target;
  /* Mechanisms */
  private Drive drive;
  private Manipulator manipulator; // Change variable name to match your class name

  private double curr_arm_target;
  
  /* Autonomous Modes */
  private Basic basic;
  private MultiNote multinote;
  private SendIt sendit;

  // PID constants should be tuned per robot
  /*final double LINEAR_P = 10;
  final double LINEAR_D = 8;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D); */

  final double ANGULAR_P = 0.0095;
  final double ANGULAR_D = 0.002;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  Command auto ;

  @Override
  public void robotInit() {
    drive = new Drive();
    manipulator = new Manipulator();
    curr_arm_target = Manipulator.kARM_START_POS; // (manipulator.getArmEnc())

    m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameCustom);
    m_chooser.addOption("Basic", "Basic");
    m_chooser.addOption("MultiNote", "MultiNote");
    m_chooser.addOption("SendIt", "SendIt");

    SmartDashboard.putData("Auto Modes", m_chooser);

    m_driveController = new PS5Controller(PS5ControllerPorts.DRIVETRAIN_PORT);
    m_manipController = new PS5Controller(PS5ControllerPorts.MANIPULATOR_PORT);

    camera = new PhotonCamera("Camera");
  }

  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }
  
  double autonomousStartTime;
  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    basic = new Basic();
    multinote = new MultiNote();
    sendit = new SendIt();
    if ("Basic".equals(m_autoSelected)) {
      auto = basic;
    } else if ("MultiNote".equals(m_autoSelected)) {
      auto = multinote;
    } else if ("SendIt".equals(m_autoSelected)) {
      sendit.run();
    } else {
      auto = basic;
    }
    //autonomousStartTime = Timer.getFPGATimestamp();
    if (auto != null){
      auto.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
    //double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    CommandScheduler.getInstance().run();

  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
    // Drive
    double power = 0;
    double steering = 0;

      
    if (m_driveController.getL2Axis() > 0.1) {
      power = m_driveController.getLeftY() * 0.8;
      steering = m_driveController.getRightX() * 0.8;
      
    } else {
      power = m_driveController.getLeftY() * 0.3;
      steering = m_driveController.getRightX() * 0.3;
    }

    if (Math.abs(power) < 0.1) {
        power = 0;
      }

    if (Math.abs(steering) < 0.1) {
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

    /* Shooter */
    if (m_manipController.getR2Axis() > 0.1) {
      if (manipulator.getArmEnc() > Manipulator.kARM_START_POS) {
        // if arm turned back farther than starting config
        manipulator.shoot(0.25);
      } else {
         NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
        // read values periodically
        double ty = table.getEntry("targetPixelsY").getDouble(0.0);
         double shotAngle = 0.0000000754886* Math.pow(ty, 2) + 0.0000615194497* ty + 0.2234508814813;
         SmartDashboard.putNumber("shotAngle", shotAngle);
         SmartDashboard.putNumber("PhotonY", ty);

         curr_arm_target = shotAngle;
        // post to smart dashboard periodically
      }
        var result = camera.getLatestResult();
        // Put the ID you want to follow or prioritize
        // int targetID = target.getFiducialId();
  
        // && targetID == 4
        if (result.hasTargets()) {

  
          steering = -turnController.calculate(result.getBestTarget().getYaw(),0);
         drive.move(power, steering);
         SmartDashboard.putNumber("moveAngle", steering);

     if (m_manipController.getL2Axis() > 0.1) {
       manipulator.intake(1.0);
     }

        }
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
      manipulator.shoot(0.0);
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

    SmartDashboard.putNumber("Arm enc", manipulator.getArmEnc());
    SmartDashboard.putNumber("D-Sensor Range", manipulator.getRange());
    SmartDashboard.putNumber("Gyro Angle", drive.getGyroAngle());
    SmartDashboard.putNumber("Arm Target", curr_arm_target);
  }
}
