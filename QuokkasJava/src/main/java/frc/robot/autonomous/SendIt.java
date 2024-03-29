package frc.robot.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class SendIt extends Command {
  private double st = 0; // start time, wrapped in an array
  private Manipulator manipulator =  new Manipulator(); // before: Manipulator manipulator = Manipulator.getInstance();
  private Drive drive = new Drive(); // before: Drive drive = new Drive.getInstance();

  public SendIt() {}

  public void run() {
    /*st = System.currentTimeMillis() / 1000.0; // Get current time in seconds
    SmartDashboard.putNumber("start time", st);

    SmartDashboard.putNumber("time", System.currentTimeMillis() / 1000.0);

    if (st < 2)) {
      manipulator.shoot(0.5);
      manipulator.armToPos(0.545);
    } else if (st < 4)) {
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (st < 5.5)) {
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.375);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, 0.0);
    } else if (st < 7.0)) {
      manipulator.intake(0.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double ty = table.getEntry("ty").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else if (st < 9.0)) {
      manipulator.intake(1.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double ty = table.getEntry("ty").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else if (st < 11.0)) {
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.45);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, -90.0);
    } else if (st < 12.0)) {
      manipulator.intake(0.0);
      manipulator.shoot(0.3);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(0.0, -46.0);
    } else if (st < 13.5)) {
      manipulator.intake(0.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double ty = table.getEntry("ty").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      double tx = table.getEntry("tx").getDouble(0.0);
      double Kp = 0.05;
      drive.move(0.0, Kp * tx);
    } else if (st < 15.0)) {
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
    }*/
  }
}
