// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.Basic;
import frc.robot.autonomous.MultiNote;
import frc.robot.autonomous.SendIt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;


import java.util.Map;

public class Robot extends TimedRobot {

    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final String kAutoNameDefault = "Default";
    private final String kAutoNameCustom = "My Auto";
    private String m_autoSelected;
    private XboxController xbox;
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

        xbox = new XboxController(0);
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
    public void teleopInit() {
    }
    
    @Override
    public void teleopPeriodic() {
      // Drive
      double power = -xbox.getRawAxis(1);     // 1 -- Left Y Axis
      double steering = xbox.getRawAxis(4);  // 4 -- Right X Axis

      drive.move(power, steering);

      // Intake
      if (xbox.getRawButton(6) && manipulator.getNoteSensor()) {  // Replace Hand.kRight with the button index (6)
          manipulator.intake(0.375);
          if (xbox.getRawAxis(3) < 0.5) {  // Replace Hand.kRight with the trigger axis (3)
              this.curr_arm_target = Manipulator.kARM_FLOOR_POS;
          }
      } else if (xbox.getRawButton(5)) {  // Replace Hand.kLeft with the button index (5)
          manipulator.intake(-1.0);
          manipulator.shoot(-0.25);
      } else {
          manipulator.intake(0.0);
          manipulator.shoot(0.0);
      }

      if (xbox.getRawButton(6) && manipulator.getNoteSensor()) {  // Replace Hand.kRight with the button index (6)
          xbox.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      } else {
          xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      }

      if (xbox.getRawButtonReleased(6)) {  // Replace Hand.kRight with the button index (6)
          this.curr_arm_target = Manipulator.kARM_FENDER_POS;
      }

      // Shooter
      if (xbox.getRawAxis(3) > 0.1) {  // Replace Hand.kRight with the trigger axis (3)
          if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
              manipulator.shoot(0.25);
          } else {
              NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
              double tx = table.getEntry("tx").getDouble(0.0);
              double Kp = 0.05;
              drive.move(power, Kp * tx);
              SmartDashboard.putNumber("tx", tx);
          }
      }

      if (xbox.getRawAxis(3) > 0.5) {  // Replace Hand.kRight with the trigger axis (3)
          if (manipulator.getArmEnc() < Manipulator.kARM_START_POS) {
              manipulator.intake(1.0);
              manipulator.shoot(0.5);
          } else {
              manipulator.shoot((xbox.getRawAxis(3) - 0.5) * 2);
          }

          if (xbox.getRawButton(6)) {  // Replace Hand.kRight with the button index (6)
              manipulator.intake(1.0);
          }
      } else {
          manipulator.shoot(0.0);
      }

      // Arm manual control
      if (xbox.getRawButtonPressed(4)) {  // Replace Hand.kY with the button index (4)
          this.curr_arm_target = Manipulator.kARM_AMP_POS;
      }

      if (xbox.getPOV(0) == 0) {
          manipulator.moveArm(0.5);  // Up
          this.curr_arm_target = manipulator.getArmEnc();
      } else if (xbox.getPOV(0) == 180) {
          manipulator.moveArm(-0.5);  // Down
          this.curr_arm_target = manipulator.getArmEnc();
      } else {
          manipulator.armToPos(curr_arm_target);
      }

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

    public static void main(String[] args) {
        Robot robot = new Robot();
        robot.startCompetition();
    }
}
