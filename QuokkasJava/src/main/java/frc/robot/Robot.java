// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PS5ControllerPorts;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.MultiNote;
import frc.robot.autonomous.SendIt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot {

  private static final String NothingAuto = "do nothing";
  private static final String Sendit = "Sendit/3notes";
  private static final String Multinote = "Multinote/2notes";
  private static final String Basic = "Basic/drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private PS5Controller m_manipController;
  private PS5Controller m_driveController;
  
  PhotonCamera camera;
  PhotonTrackedTarget target;
  /* Mechanisms */
  private Drive drive;
  private Manipulator manipulator; // Change variable name to match your class name

  private double curr_arm_target;

  /* Autonomous Modes */
  private Basic basic = new Basic();
  private MultiNote multinote = new MultiNote();
  private SendIt sendit = new SendIt();

  static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20); 
  static final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.4);
  static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);



  // PID constants should be tuned per robot
  // TODO: Tune the PID.
  final double LINEAR_P = 10;
  final double LINEAR_D = 8;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 10;
  final double ANGULAR_D = 8;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  @Override
  public void robotInit() {
    drive = Drive.getInstance();
    manipulator = Manipulator.getInstance();
    curr_arm_target = Manipulator.kARM_START_POS; // TODO: CONFIGURE THE POSITIONS FOR THE ENCODER
    // (manipulator.get())

    m_chooser.setDefaultOption(NothingAuto, NothingAuto);
    
    SmartDashboard.putData("Auto Modes", m_chooser);

    m_driveController = new PS5Controller(PS5ControllerPorts.DRIVETRAIN_PORT);
    m_manipController = new PS5Controller(PS5ControllerPorts.MANIPULATOR_PORT);

    camera = new PhotonCamera("Camera");
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }  
  

  double AUTO_DRIVE_SPEED;
  double AUTO_LAUNCHER_SPEED;
  double AUTO_INTAKE_SPEED;
  double AUTO_ARM_SPEED;

  double autonomousStartTime;

  @Override
  public void autonomousInit() {
   m_autoSelected = m_chooser.getSelected();
   System.out.println("Auto selected: " + m_autoSelected);


    AUTO_DRIVE_SPEED = -0.5;
    AUTO_LAUNCHER_SPEED = 1;
    AUTO_INTAKE_SPEED = 1;
    AUTO_ARM_SPEED = 1;

    if(m_autoSelected == Basic)
    {
      AUTO_LAUNCHER_SPEED = 0;
      AUTO_INTAKE_SPEED = 0;
      AUTO_ARM_SPEED = 0;
    }
    else if(m_autoSelected == NothingAuto)
    {
      AUTO_DRIVE_SPEED = 0;
      AUTO_LAUNCHER_SPEED = 0;
      AUTO_INTAKE_SPEED = 0;
      AUTO_ARM_SPEED = 0;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (m_autoSelected == Basic) {
      basic.run();
    } else if (m_autoSelected == Multinote) {
      if (timeElapsed < 2.0) {
      // Lower arm
      manipulator.shoot(0.00);
      manipulator.armToPos(0.00);
    } else if (timeElapsed < 4.0) {
      // Shoot
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (timeElapsed < 5.5) {
      // Drive, intake
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.375);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, 0.0);
    } else if (timeElapsed < 7.0) {
      // Drive back
      manipulator.intake(0.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(-0.5, 0.0);
    } else if (timeElapsed < 9.0) {
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

    } else if (m_autoSelected == Sendit) {
      sendit.run();
    } else {
      basic.run();
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Drive
    double power;
    double steering;
    double armEnc;
    double armpower;
    double armDis;
    double x;

    power = m_driveController.getRightX() * 0.6;
    if (Math.abs(power) < 0.1) {
      power = 0;
    }

    // If square pressed, aligns
    if (m_driveController.getSquareButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();
      // Put the ID you want to follow or prioritize
      // int targetID = target.getFiducialId();

      // && targetID == 4
      if (result.hasTargets()) {
        // First calculate range
        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort increases yaw
        steering =
            turnController.calculate(
                result.getBestTarget().getYaw(), 0); // TODO: Change positive or negative by trying
      } else {
        // If we have no targets, stay still.
        steering = 0;
      }
    } else {
      // Manual Driver Mode
      steering = m_driveController.getLeftY() * 0.6;

      if (Math.abs(steering) < 0.1) {
        steering = 0;
      }
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
        /** High goal shooting, Set automatic shot angle */
       var result = camera.getLatestResult();
       if (result.hasTargets()) {

        double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
        // First calculate range
        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort increases yaw
        armDis =  0.95;
        armEnc = 0.235;
        armpower = range * armEnc;
        x = armpower / armDis;

        //95 cm + 0.95m
        //0.235 enc
        SmartDashboard.putNumber("X value", x);
        SmartDashboard.putNumber("range", range);

        manipulator.armToPos(x);
        

      } else {
        // If we have no targets, stay still.
        steering = 0;
      }
    }
  }

    /* Vision aiming section */

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
        manipulator.shoot(0.0);
      }

  

    /*Arm manual control*/
    if (m_manipController.getTriangleButtonPressed()) {
      // Amp stating config
      curr_arm_target = Manipulator.kARM_AMP_POS;
    }

    if (m_manipController.getPOV(0) == 0) {
      manipulator.moveArm(0.15); // Up
      curr_arm_target = manipulator.getArmEnc();
    } else if (m_manipController.getPOV(0) == 180) {
      manipulator.moveArm(-0.15); // Down
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
