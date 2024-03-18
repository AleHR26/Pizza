package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SendIt {
  private Manipulator manipulator = new Manipulator();
  private Drive drive = new Drive();
  private Timer autotime = PhotonVisionConstants.autotime;

  PhotonCamera camera;
  PhotonTrackedTarget target;
  PIDController turnController =
      new PIDController(PhotonVisionConstants.ANGULAR_P, 0, PhotonVisionConstants.ANGULAR_D);

  public SendIt() {
    autotime.start();
    drive.zeroGyro();
    autotime.reset();
  }

  public void run() {
    if (autotime.get() < 2) {
      manipulator.shoot(0.5);
      manipulator.armToPos(0.079);
    } else if (autotime.get() < 4) {
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (autotime.get() < 5.5) {
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.375);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, 0.0);
    } else if (autotime.get() < 7.0) {
      manipulator.intake(0.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
      double ty = table.getEntry("targetPixelsY").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      var result = camera.getLatestResult();
      double goalTarget = 0;

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == 4) // change to 7 for blue side
          {
            goalTarget = targets.get(i).getYaw();
          }
        }
        drive.move(0, -turnController.calculate(goalTarget, 0));
      }
    } else if (autotime.get() < 9.0) {
      manipulator.intake(1.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
      double ty = table.getEntry("targetPixelsY").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      var result = camera.getLatestResult();
      double goalTarget = 0;

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == 4) // change to 7 for blue side
          {
            goalTarget = targets.get(i).getYaw();
          }
        }
        drive.move(0, -turnController.calculate(goalTarget, 0));
      }
    } else if (autotime.get() < 11.0) {
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.45);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, -90.0);
    } else if (autotime.get() < 12.0) {
      manipulator.intake(0.0);
      manipulator.shoot(0.3);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(0.0, -46.0);
    } else if (autotime.get() < 13.5) {
      manipulator.intake(0.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
      double ty = table.getEntry("targetPixelsY").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);

      var result = camera.getLatestResult();
      double goalTarget = 0;

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == 4) // change to 7 for blue side
          {
            goalTarget = targets.get(i).getYaw();
          }
        }
        drive.move(0, -turnController.calculate(goalTarget, 0));
      }
    } else if (autotime.get() < 15.0) {
      manipulator.intake(1.0);
      manipulator.shoot(1.0);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Camera");
      double ty = table.getEntry("targetPixelsY").getDouble(0.0);
      double shot_angle = -0.00008 * Math.pow(ty, 2) + 0.00252 * ty + 0.4992;
      manipulator.armToPos(shot_angle);
      // manipulator.armToPos(0.024);

      var result = camera.getLatestResult();
      double goalTarget = 0;

      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (int i = 0; i < targets.size(); i++) {
          if (targets.get(i).getFiducialId() == 4) // change to 7 for blue side
          {
            goalTarget = targets.get(i).getYaw();
          }
        }
        drive.move(0, -turnController.calculate(goalTarget, 0));
      }
    } else {
      drive.move(0.0, 0.0);
      manipulator.intake(0.0);
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    }
  }
}
