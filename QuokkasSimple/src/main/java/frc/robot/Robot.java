// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.kauailabs.navx.frc.AHRS;
// Imports that allow the usage of REV Spark Max motor controllers
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Manipulator;


public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kMultinote = "launch drive";
  private static final String kSendit = "launch";
  private static final String kBasic = "drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Manipulator manipulator;
  private double curr_arm_target;
  /*
   * Drive motor controller instances.
   *
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are uisng NEOs.
   * The rookie kit comes with CIMs which are brushed motors.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkBase leftRear = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkBase leftFront = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkBase rightRear = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkBase rightFront = new CANSparkMax(4, MotorType.kBrushless);

  /*
   * A class provided to control your drivetrain. Different drive styles can be passed to differential drive:
   * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java
   */
  DifferentialDrive m_drivetrain;


  AHRS navx = new AHRS(SPI.Port.kMXP);

  /*
   * Launcher motor controller instances.
   *
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (VictorSPX) to match your robot as necessary.
   *
   * Both of the motors used on the KitBot launcher are CIMs which are brushed motors
   */
  
  PhotonCamera camera;

  /**
   * Roller Claw motor controller instance.
  */
  /**
  
    /**
   * The starter code uses the most generic joystick class.
   *
   * To determine which button on your controller corresponds to which number, open the FRC
   * driver station, go to the USB tab, plug in a controller and see which button lights up
   * when pressed down
   *
   * Buttons index from 0
   */
  Joystick m_driverController = new Joystick(0);


  Joystick m_manipController = new Joystick(1);

  


  // --------------- Magic numbers. Use these to adjust settings. ---------------
  // Constants such as camera and target height stored. Change per robot and goal!
  static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
  static final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.4);
  // Angle between horizontal and the camera.
  static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  static final double GOAL_RANGE_METERS = Units.feetToMeters(2);

  // PID constants should be tuned per robot
  final double LINEAR_P = 10;
  final double LINEAR_D = 8;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 10;
  final double ANGULAR_D = 8;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  
 /**
   * How many amps can an individual drivetrain motor use.
   */
  static final int DRIVE_CURRENT_LIMIT_A = 60;

  /**
   * How many amps the feeder motor can use.
   */
  static final int FEEDER_CURRENT_LIMIT_A = 60;

  /**
   * Percent output to run the feeder when expelling note
   */
  static final double FEEDER_OUT_SPEED = .4;

  /**
   * Percent output to run the feeder when intaking note
   */
  static final double FEEDER_IN_SPEED = -.4;

  /**
   * Percent output for amp or drop note, configure based on polycarb bend
   */
  static final double FEEDER_AMP_SPEED = .4;

  /**
   * How many amps the launcher motor can use.
   *
   * In our testing we favored the CIM over NEO, if using a NEO lower this to 60
   */
  static final int LAUNCHER_CURRENT_LIMIT_A = 60;

  /**
   * How many amps te arm motor can use
   */
  static final int ARM_CURRENT_LIMIT_A = 60;

  /**
   * Percent output to run the launcher when intaking AND expelling note
   */
  static final double LAUNCHER_SPEED = 1.0;

  /**
   * Percent output for scoring in amp or dropping note, configure based on polycarb bend
   * .14 works well with no bend from our testing
   */
  static final double LAUNCHER_AMP_SPEED = .17;
  /**
   * Percent output for the roller claw
   */
  static final double CLAW_OUTPUT_POWER = .5;
  /**
   * Percent output to help retain notes in the claw
   */
  static final double CLAW_STALL_POWER = .1;
  /**
   * Percent output to power the climber
   */
  static final double CLIMER_OUTPUT_POWER = 1;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    manipulator = Manipulator.getInstance();
    this.curr_arm_target = Manipulator.kARM_START_POS;

    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("launch note and drive", kMultinote);
    m_chooser.addOption("launch", kSendit);
    m_chooser.addOption("drive", kBasic);
    SmartDashboard.putData("Auto choices", m_chooser);

    camera = new PhotonCamera("Camera");

    /*
     * Apply the current limit to the drivetrain motors
     */
    leftRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    leftFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    rightRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    rightFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);

    /*
     * Tells the rear wheels to follow the same commands as the front wheels
     */
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    /*
     * One side of the drivetrain must be inverted, as the motors are facing opposite directions
     */
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    rightFront.setInverted(false);
    rightRear.setInverted(false);

    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    /*
     * Launcher wheel(s) spinning the wrong direction? Change to true here.
     *
     * Add white tape to wheel to help determine spin direction.
     */

   

    /*
     * Apply the current limit to the launching mechanism
     */
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test modes.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }


  /*
   * Auto constants, change values below in autonomousInit()for different autonomous behaviour
   *
   * A delayed action starts X seconds into the autonomous period
   *
   * A time action will perform an action for X amount of seconds
   *
   * Speeds can be changed as desired and will be set to 0 when
   * performing an auto that does not require the system
   */
  double AUTO_LAUNCH_DELAY_S;
  double AUTO_DRIVE_DELAY_S;

  double AUTO_DRIVE_TIME_S;

  double AUTO_DRIVE_SPEED;
  double AUTO_LAUNCHER_SPEED;

  double autonomousStartTime;

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    leftRear.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);

    AUTO_LAUNCH_DELAY_S = 2;
    AUTO_DRIVE_DELAY_S = 3;

    AUTO_DRIVE_TIME_S = 2.0;
    AUTO_DRIVE_SPEED = -0.5;
    AUTO_LAUNCHER_SPEED = 1;
    
    /*
     * Depeding on which auton is selected, speeds for the unwanted subsystems are set to 0
     * if they are not used for the selected auton
     *
     * For kDrive you can also change the kAutoDriveBackDelay
     */
    if(m_autoSelected == kSendit)
    {
      AUTO_DRIVE_SPEED = 0;
    }
    else if(m_autoSelected == kBasic)
    {
      AUTO_LAUNCHER_SPEED = 0;
    }
    else if(m_autoSelected == kNothingAuto)
    {
      AUTO_DRIVE_SPEED = 0;
      AUTO_LAUNCHER_SPEED = 0;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;

        if (m_manipController.getRawButton(1)) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();
            

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters( 
                                CAMERA_HEIGHT_METERS, 
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS, 
                                Units.degreesToRadians(result.getBestTarget().getPitch()));
               

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range

                forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            forwardSpeed = -m_driverController.getRawAxis(1);
            rotationSpeed = -m_driverController.getRawAxis(2);
           
          }

    /*
     * Spins up the launcher wheel
     */
    if (m_manipController.getRawButton(6) && manipulator.getNoteSensor()) {
      // If pressing intake button, and the NOTE is not in the intake
      manipulator.intake(0.375);
      if (m_manipController.getRawAxis(3) < 0.5) {
        this.curr_arm_target = Manipulator.kARM_FLOOR_POS;
      }
    } else if (m_manipController.getRawButton(5)) {
      // Outtake
      manipulator.shoot(-0.25);
    } else {
      // Do nothing
      manipulator.intake(0.0);
      manipulator.shoot(0.0);
    }

    if (m_manipController.getRawButton(6) && manipulator.getNoteSensor()) {
      // If pressing intake and NOTE is in the intake
      m_manipController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    } else {
      m_manipController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    if (m_manipController.getRawButtonReleased(6)) {
      // No longer intaking; raise intake to avoid damage
      this.curr_arm_target = Manipulator.kARM_FENDER_POS;
    }

    if (m_manipController.getRawAxis(3) > 0.5) {
      if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
        // If arm turned back farther than starting config, score AMP
        manipulator.intake(1.0);
        manipulator.shoot(0.5);
      } else {
        // High goal shooting
        // Adjustable by driver. 50% press => 0% power, 100% press => 100% power
        manipulator.shoot((m_manipController.getRawAxis(3) - 0.5) * 2);
      }

      if (m_manipController.getRawButton(6)) {
        // Run intake despite NOTE being in intake
        manipulator.intake(1.0);
      }
    } else {
      manipulator.shoot(0.0);
    }


    // Arm manual control
    if (m_manipController.getRawButtonPressed(4)) {
      // Amp scoring config
      this.curr_arm_target = Manipulator.kARM_AMP_POS;
    }

    /*
     * While the button is being held spin both motors to intake note
     */
    if (m_manipController.getRawButtonReleased(6)) {
      // No longer intaking; raise intake to avoid damage
      this.curr_arm_target = Manipulator.kARM_FENDER_POS;
    }

    if (m_manipController.getPOV(0) == 0) {
      manipulator.moveArm(0.5); // Up
      this.curr_arm_target = manipulator.getArmEnc();
    } else if (m_manipController.getPOV(0) == 180) {
      manipulator.moveArm(-0.5); // Down
      this.curr_arm_target = manipulator.getArmEnc();
    } else {
      // Move arm to preset target, or current position if last command was manual control.
      manipulator.armToPos(curr_arm_target);
    }

    SmartDashboard.putNumber("Arm", manipulator.getArmEnc());
    

    m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed, false);
  }
}

/*
 * The kit of parts drivetrain is known as differential drive, tank drive or skid-steer drive.
 *
 * There are two common ways to control this drivetrain: Arcade and Tank
 *
 * Arcade allows one stick to be pressed forward/backwards to power both sides of the drivetrain to move straight forwards/backwards.
 * A second stick (or the second axis of the same stick) can be pushed left/right to turn the robot in place.
 * When one stick is pushed forward and the other is pushed to the side, the robot will power the drivetrain
 * such that it both moves fowards and turns, turning in an arch.
 *
 * Tank drive allows a single stick to control of a single side of the robot.
 * Push the left stick forward to power the left side of the drive train, causing the robot to spin around to the right.
 * Push the right stick to power the motors on the right side.
 * Push both at equal distances to drive forwards/backwards and use at different speeds to turn in different arcs.
 * Push both sticks in opposite directions to spin in place.
 *
 * arcardeDrive can be replaced with tankDrive like so:
 *
 * m_drivetrain.tankDrive(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(5))
 *
 * Inputs can be squared which decreases the sensitivity of small drive inputs.
 *
 * It literally just takes (your inputs * your inputs), so a 50% (0.5) input from the controller becomes (0.5 * 0.5) -> 0.25
 *
 * This is an option that can be passed into arcade or tank drive:
 * arcadeDrive(double xSpeed, double zRotation, boolean squareInputs)
 *
 *
 * For more information see:
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
 *
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java
 *
 */
