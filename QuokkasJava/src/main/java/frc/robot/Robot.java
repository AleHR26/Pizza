// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.autonomous.MultiNote;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.urcl.URCL;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.SendIt;

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

  private Timer autotime = PhotonVisionConstants.autotime;
   /* Autonomous Modes */
   private Basic basic;
   private MultiNote multinote;
   private SendIt sendit;


  // PID constants should be tuned per robot
  // TODO: Tune the PID.

  PIDController turnController =
      new PIDController(PhotonVisionConstants.ANGULAR_P, 0, PhotonVisionConstants.ANGULAR_D);

  @Override
  public void robotInit() {
    drive = Drive.getInstance();
    manipulator = Manipulator.getInstance();
    curr_arm_target = Manipulator.kARM_START_POS; // TODO: CONFIGURE THE POSITIONS FOR THE ENCODER
    // (manipulator.get())

    m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.addOption("Basic", "Basic");
    m_chooser.addOption("MultiNote", "MultiNote");
    m_chooser.addOption("SendIt", "SendIt");

    SmartDashboard.putData("Auto Modes", m_chooser);

    m_driveController = new PS5Controller(PS5ControllerPorts.DRIVETRAIN_PORT);
    m_manipController = new PS5Controller(PS5ControllerPorts.MANIPULATOR_PORT);

    camera = new PhotonCamera("Camera");
    drive.zeroGyro();

    DataLogManager.start();
    URCL.start();
  }

  @Override
  public void robotPeriodic() {
    double matchTime = Timer.getMatchTime();
    double currentTimeStamp = Timer.getFPGATimestamp();
    double dt = currentTimeStamp - lastTimestamp;
    SmartDashboard.putNumber("Gyro", drive.getGyroAngle());

    lastTimestamp = currentTimeStamp;
  }

  private boolean testinit;

  @Override
  public void autonomousInit() {
    autotime.start();

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    basic = new Basic();
    multinote = new MultiNote();
    sendit = new SendIt();
    drive.zeroGyro();
    autotime.reset();
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
    basic.run();;
    }

  }

  @Override
  public void teleopInit() {
    drive.zeroGyro();
  }

  @Override
  public void teleopPeriodic() {

    int ApriltagID = 0;

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {

      ApriltagID = 4;
       
    }
    if (ally.get() == Alliance.Blue) {

      ApriltagID = 7;
        
    }
    }
    else {
      ApriltagID = 7;
    
      }

    drive.m_drivetrain.feed();

    if (m_driveController.getCrossButton()) {
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    } else if (m_driveController.getTriangleButton()) {
      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
    } else if (m_driveController.getCircleButton()) {
      drive.sysIdDynamic(SysIdRoutine.Direction.kForward);
    } else if (m_driveController.getSquareButton()) {
      drive.sysIdDynamic(SysIdRoutine.Direction.kReverse);
    }

    // Drive
    double power = 0;
    double steering = 0;

    if (m_driveController.getR2Axis() > 0.1) {
      power = m_driveController.getLeftY() * 1;
      steering = m_driveController.getRightX() * 1;

    } else {
      power = m_driveController.getLeftY() * 0.5;
      steering = m_driveController.getRightX() * 0.5;
    }

    if (m_driveController.getL2Axis() > 0.1) {
      power = m_driveController.getLeftY() * 0.25;
      steering = m_driveController.getRightX() * 0.25;

    } else {
      power = m_driveController.getLeftY() * 0.5;
      steering = m_driveController.getRightX() * 0.5;
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
        double shotAngle =
            -0.00000000562323 * Math.pow(ty, 2) + 0.0001606189982 * ty + 0.2199664090903;
        SmartDashboard.putNumber("shotAngle", shotAngle);
        // double FarAngle = 0; // Function with far shots
        SmartDashboard.putNumber("PhotonY", ty);
        // curr_arm_target = shotAngle;
        // curr_arm_target = shotAngle;
        // if (ty > 600) {
        //  curr_arm_target = shotAngle;
        // } else {
        // curr_arm_target = FarAngle;
        // }

        // post to smart dashboard periodically
      }
      var result = camera.getLatestResult();
      // Put the ID you want to follow or prioritize
      // int targetID = target.getFiducialId();

      // && targetID == ApriltagID
      double goalTarget = 0;

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        // int i = 0;

        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == ApriltagID) // change to 7 for blue side
          {
            goalTarget = targets.get(i).getYaw();
          }
        }

        steering = -turnController.calculate(goalTarget, 0);
        // double cheap = steering;
        drive.move(power, steering);
        SmartDashboard.putNumber("moveAngle", steering);
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
