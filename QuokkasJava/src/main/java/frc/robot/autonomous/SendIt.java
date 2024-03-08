package frc.robot.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Util;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class SendIt {
  // start time, wrapped in an array
  Manipulator manipulator = new Manipulator(); // before: Manipulator manipulator = Manipulator.getInstance();
  Drive drive = new Drive(); // before: Drive drive = new Drive.getInstance();
  double autonomousStartTime = Timer.getFPGATimestamp();
  public SendIt() {}

  public void run() {
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    
    if (timeElapsed < 2) {
      manipulator.shoot(0.5);
      manipulator.armToPos(0.545);
    } else if (timeElapsed < 4) {
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (timeElapsed < 5.5) {
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.375);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, 0.0);
    } else if (timeElapsed < 7.0) {
      manipulator.intake(0.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double ty = table.getEntry("ty").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else if (timeElapsed < 9.0) {
      manipulator.intake(1.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double ty = table.getEntry("ty").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else if (timeElapsed < 11.0) {
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.45);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, -90.0);
    } else if (timeElapsed < 12.0) {
      manipulator.intake(0.0);
      manipulator.shoot(0.3);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(0.0, -46.0);
    } else if (timeElapsed < 13.5) {
      manipulator.intake(0.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double ty = table.getEntry("ty").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else if (timeElapsed < 15.0) {
      manipulator.intake(1.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      manipulator.armToPos(0.490);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else {
      drive.move(0.0, 0.0);
      manipulator.intake(0.0);
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    }
  }
}
