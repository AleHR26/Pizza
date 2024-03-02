package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.Util;

public class SendIt {
    private double[] st = { 0.0 }; // start time, wrapped in an array

    public SendIt() {
    }

    public void run() {
        Manipulator manipulator = Manipulator.getInstance();
        Drive drive = Drive.getInstance();

        st[0] = System.currentTimeMillis() / 1000.0; // Get current time in seconds
        SmartDashboard.putNumber("start time", st[0]);

        SmartDashboard.putNumber("time", System.currentTimeMillis() / 1000.0);

        if (Util.wait(st, 2)) {
            manipulator.shoot(0.5);
            manipulator.armToPos(0.545);
        } else if (Util.wait(st, 4)) {
            manipulator.intake(1.0);
            manipulator.shoot(0.5);
            manipulator.armToPos(Manipulator.kARM_FENDER_POS);
        } else if (Util.wait(st, 5.5)) {
            if (manipulator.getNoteSensor()) {
                manipulator.intake(0.375);
            } else {
                manipulator.intake(0.0);
            }
            manipulator.shoot(0.0);
            manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
            drive.gyroDrive(0.375, 0.0);
        } else if (Util.wait(st, 7.0)) {
            manipulator.intake(0.0);
            manipulator.shoot(1.0);

            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double ty = table.getEntry("ty").getDouble(0.0);
            double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
            manipulator.armToPos(shot_angle);

            double tx = table.getEntry("tx").getDouble(0.0);
            double Kp = 0.05;
            drive.move(0.0, Kp * tx);
        } else if (Util.wait(st, 9.0)) {
            manipulator.intake(1.0);
            manipulator.shoot(1.0);

            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double ty = table.getEntry("ty").getDouble(0.0);
            double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
            manipulator.armToPos(shot_angle);

            double tx = table.getEntry("tx").getDouble(0.0);
            double Kp = 0.05;
            drive.move(0.0, Kp * tx);
        } else if (Util.wait(st, 11.0)) {
            if (manipulator.getNoteSensor()) {
                manipulator.intake(0.45);
            } else {
                manipulator.intake(0.0);
            }
            manipulator.shoot(0.0);
            manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
            drive.gyroDrive(0.375, -90.0);
        } else if (Util.wait(st, 12.0)) {
            manipulator.intake(0.0);
            manipulator.shoot(0.3);
            manipulator.armToPos(Manipulator.kARM_FENDER_POS);
            drive.gyroDrive(0.0, -46.0);
        } else if (Util.wait(st, 13.5)) {
            manipulator.intake(0.0);
            manipulator.shoot(1.0);

            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double ty = table.getEntry("ty").getDouble(0.0);
            double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
            manipulator.armToPos(shot_angle);

            double tx = table.getEntry("tx").getDouble(0.0);
            double Kp = 0.05;
            drive.move(0.0, Kp * tx);
        } else if (Util.wait(st, 15.0)) {
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
