// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.MultiNote;
import frc.robot.autonomous.SendIt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.Util;


import java.util.Map;

public class Robot extends TimedRobot {

    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final String kAutoNameDefault = "Default";
    private final String kAutoNameCustom = "My Auto";
    private String m_autoSelected;
    private PS5Controller PS5 = new PS5Controller(0);
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
        curr_arm_target = Manipulator.kARM_START_POS;

        m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
        m_chooser.addOption("Basic", "Basic");
        m_chooser.addOption("MultiNote", "MultiNote");
        m_chooser.addOption("SendIt", "SendIt");

        SmartDashboard.putData("Auto Modes", m_chooser);
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
    public void teleopInit() {
    }
    
    @Override
    public void teleopPeriodic() {
      // Drive

      double power = -PS5.getRawAxis(2)*0.7;     // 1 -- Left Y Axis
      double steering = -PS5.getRawAxis(1)*0.7;  // 4 -- Right X Axis

     if (Math.abs(steering)< 0.1) {
        steering = 0; 
     }
     if (Math.abs(power)< 0.1) { 
      power = 0;
     }
     drive.move(power, steering);

      // Intake
      if (PS5.getR1Button() && manipulator.getNoteSensor()) { 
          manipulator.intake(0.375);
          if (PS5.getR2Axis() < 0.5) {  // Replace Hand.kRight with the trigger axis (3)
              this.curr_arm_target = Manipulator.kARM_FLOOR_POS;
          }
      } else if (PS5.getL1Button()) {  // Replace Hand.kLeft with the button index (5)
          manipulator.intake(-1.0);
          manipulator.shoot(-0.25);
      } else {
          manipulator.intake(0.0);
          manipulator.shoot(0.0);
      }

      if (PS5.getR1Button() && manipulator.getNoteSensor()) {  // Replace Hand.kRight with the button index (6)
          PS5.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      } else {
          PS5.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      }

      if (PS5.getR1ButtonReleased()) {  // Replace Hand.kRight with the button index (6)
          this.curr_arm_target = Manipulator.kARM_FENDER_POS;
      }

      // Shooter
      if (PS5.getR2Axis() > 0.1) {  // Replace Hand.kRight with the trigger axis (3)
          if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
              manipulator.shoot(0.25);
          } else {
              //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
              //double tx = table.getEntry("tx").getDouble(0.0);
              //double Kp = 0.05;
              //drive.move(power, Kp * tx);
              //SmartDashboard.putNumber("tx", tx);
          }
      }

      if (PS5.getR2Axis() > 0.5) {  // Replace Hand.kRight with the trigger axis (3)
          if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
              manipulator.intake(1.0);
              manipulator.shoot(0.5);
          } else {
              manipulator.shoot((PS5.getR2Axis() - 0.5) * 2);
          }

          if (PS5.getR1Button()) {  // Replace Hand.kRight with the button index (6)
              manipulator.intake(1.0);
          }
      } else {
          manipulator.shoot(0.0);
      }/* */

       //Arm manual control
      if (PS5.getTriangleButtonPressed()) {  
          curr_arm_target = Manipulator.kARM_AMP_POS;
      }

      if (PS5.getPOV(0) == 0) {
          manipulator.moveArm(0.1);  // Up
          curr_arm_target = manipulator.getArmEnc();
      } else if (PS5.getPOV(0) == 180) {
          manipulator.moveArm(-0.1); // Down
          curr_arm_target = manipulator.getArmEnc();
      } else {
          manipulator.armToPos(curr_arm_target);
      }/* */

      SmartDashboard.putNumber("Arm", manipulator.getArmEnc());
  }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
