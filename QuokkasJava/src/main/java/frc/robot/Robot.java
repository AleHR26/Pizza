// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.MultiNote;
import frc.robot.autonomous.SendIt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class Robot extends TimedRobot {

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String kAutoNameDefault = "Default";
  private final String kAutoNameCustom = "My Auto";
  private String m_autoSelected;
  private Joystick ps5;
  private Timer autoTimer;
  private double lastTimestamp; // Add this line for missing variable

  /* Mechanisms */
  private Drive drive;
  private Manipulator manipulator; // Change variable name to match your class name

  private double curr_arm_target;

  /* Autonomous Modes */
  private Basic basic;
  private MultiNote multinote;
  private SendIt sendit;

  @Override
  public void robotInit() {
    drive = Drive.getInstance();
    manipulator = Manipulator.getInstance();
    this.curr_arm_target = Manipulator.kARM_START_POS;

    m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameCustom);
    m_chooser.addOption("Basic", "Basic");
    m_chooser.addOption("MultiNote", "MultiNote");
    m_chooser.addOption("SendIt", "SendIt");

    SmartDashboard.putData("Auto Modes", m_chooser);

    ps5 = new Joystick(0);
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

    this.basic = new Basic();
    this.multinote = new MultiNote();
    this.sendit = new SendIt();
  }

  @Override
  public void autonomousPeriodic() {
    if (testinit) {
      drive.zeroGyro();
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
    double power = -ps5.getRawAxis(1); // 1 -- Left Y Axis
    double steering = -ps5.getRawAxis(2); // 2 -- Right X Axis

    drive.move(power, steering);

    // Intake
    if (ps5.getRawButton(6) && manipulator.getNoteSensor()) {
      // If pressing intake button, and the NOTE is not in the intake
      manipulator.intake(0.375);
      if (ps5.getRawAxis(3) < 0.5) {
        this.curr_arm_target = Manipulator.kARM_FLOOR_POS;
      }
    } else if (ps5.getRawButton(5)) {
      // Outtake
      manipulator.shoot(-0.25);
    } else {
      // Do nothing
      manipulator.intake(0.0);
      manipulator.shoot(0.0);
    }

    if (ps5.getRawButton(6) && manipulator.getNoteSensor()) {
      // If pressing intake and NOTE is in the intake
      ps5.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    } else {
      ps5.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    if (ps5.getRawButtonReleased(6)) {
      // No longer intaking; raise intake to avoid damage
      this.curr_arm_target = Manipulator.kARM_FENDER_POS;
    }

    // Shooter
    if (ps5.getRawAxis(3) > 0.1) { // Replace Hand.kRight with the trigger axis (3)
      if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
        // If arm turned back farther than starting config
        manipulator.shoot(0.25);
      } else {
        // High goal shooting
        // Set shot angle
        // std::shared_ptr<nt::NetworkTable> table =
        // nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        // double ty = table->GetNumber("ty", 0.0);
        // double shot_angle = -0.00008 * pow(ty,2) + .00252*ty + .4992;
        // this->curr_arm_target = shot_angle;
      }
      /*vision aiming
       * NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
         double tx = table.getEntry("tx").getDouble(0.0);
         double Kp = 0.05;
         drive.move(power, Kp * tx);
         SmartDashboard.putNumber("tx", tx);
      */
    }

    if (ps5.getRawAxis(3) > 0.5) {
      if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
        // If arm turned back farther than starting config, score AMP
        manipulator.intake(1.0);
        manipulator.shoot(0.5);
      } else {
        // High goal shooting
        // Adjustable by driver. 50% press => 0% power, 100% press => 100% power
        manipulator.shoot((ps5.getRawAxis(3) - 0.5) * 2);
      }

      if (ps5.getRawButton(6)) {
        // Run intake despite NOTE being in intake
        manipulator.intake(1.0);
      }
    } else {
      manipulator.shoot(0.0);
    }

    // Arm manual control
    if (ps5.getRawButtonPressed(4)) {
      // Amp scoring config
      this.curr_arm_target = Manipulator.kARM_AMP_POS;
    }

    if (ps5.getPOV(0) == 0) {
      manipulator.moveArm(0.5); // Up
      this.curr_arm_target = manipulator.getArmEnc();
    } else if (ps5.getPOV(0) == 180) {
      manipulator.moveArm(-0.5); // Down
      this.curr_arm_target = manipulator.getArmEnc();
    } else {
      // Move arm to preset target, or current position if last command was manual control.
      manipulator.armToPos(curr_arm_target);
    }

    SmartDashboard.putNumber("Arm", manipulator.getArmEnc());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
