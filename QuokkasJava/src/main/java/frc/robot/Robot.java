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
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.MultiNote;
import frc.robot.autonomous.SendIt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot {

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String kAutoNameDefault = "Default";
  private final String kAutoNameCustom = "My Auto";
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

  // Constants such as camera and target height stored. Change per robot and goal.
  static final double CAMERA_HEIGHT_METERS =
      Units.inchesToMeters(20); // TODO: Change Photon constants to Margarita
  static final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.4);

  /** Angle between horizontal and the camera. */
  static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  /** How far from the target we want to be */
  static final double GOAL_RANGE_METERS = Units.feetToMeters(2);

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

    m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameCustom);
    m_chooser.addOption("Basic", "Basic");
    m_chooser.addOption("MultiNote", "MultiNote");
    m_chooser.addOption("SendIt", "SendIt");

    SmartDashboard.putData("Auto Modes", m_chooser);

    m_driveController = new PS5Controller(0);
    m_manipController = new PS5Controller(0);

    camera = new PhotonCamera("Camera");
  }

  @Override
  public void robotPeriodic() {
    double matchTime = Timer.getMatchTime();
    double currentTimeStamp = Timer.getFPGATimestamp();
    double dt = currentTimeStamp - lastTimestamp;

    lastTimestamp = currentTimeStamp;
  }

  private boolean testinit;

  @Override
  public void autonomousInit() {
    testinit = true;

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    basic = new Basic();
    multinote = new MultiNote();
    sendit = new SendIt();
  }

  @Override
  public void autonomousPeriodic() {
    if (testinit) {
      //drive.zeroGyro();
      testinit = false;
    }

    if ("Basic".equals(m_autoSelected)) {
      basic.run();
    } else if ("MultiNote".equals(m_autoSelected)) {
      multinote.run();
    } else if ("SendIt".equals(m_autoSelected)) {
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

    // If square pressed, aligns
    if (m_driveController.getSquareButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();
      // Put the ID you want to follow or prioritize
      //int targetID = target.getFiducialId();
      
      //&& targetID == 4
      if (result.hasTargets()) {
        // First calculate range
        double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort increases range

        power =
            forwardController.calculate(
                range, GOAL_RANGE_METERS); // TODO: Change positive or negative by trying

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort increases yaw
        steering =
            turnController.calculate(
                result.getBestTarget().getYaw(), 0); // TODO: Change positive or negative by trying
      } else {
        // If we have no targets, stay still.
        power = 0;
        steering = 0;
      }
    } else {
      // Manual Driver Mode
      power = m_driveController.getRightX() * 0.7;
      steering = m_driveController.getLeftY() * 0.7;

      if (Math.abs(steering) < 0.1) {
        steering = 0;
      }
      if (Math.abs(power) < 0.1) {
        power = 0;
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
        /** High goal shooting Set shot angle */

        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        // double tx = table.getEntry("tx").getDouble(0.0);
        // double Kp = 0.05;
        // drive.move(power, Kp * tx);
        // SmartDashboard.putNumber("tx", tx);
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
      // DO nothing
      manipulator.shoot(0.0);
    }

    /*Arm manual control*/
    if (m_manipController.getTriangleButtonPressed()) {
      // Amp stating config
      curr_arm_target = Manipulator.kARM_AMP_POS;
    }

    if (m_manipController.getPOV(0) == 0) {
      manipulator.moveArm(0.3); // Up
      curr_arm_target = manipulator.getArmEnc();
    } else if (m_manipController.getPOV(0) == 180) {
      manipulator.moveArm(-0.3); // Down
      curr_arm_target = manipulator.getArmEnc();
    } else {
      manipulator.armToPos(curr_arm_target);
    }

    SmartDashboard.putNumber("Arm enc", manipulator.getArmEnc());
    SmartDashboard.putNumber("D-Sensor Range", manipulator.getRange());
    SmartDashboard.putNumber("Gyro Angle", drive.getGyroAngle());
  }
}