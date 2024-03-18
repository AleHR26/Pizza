package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class Basic {
  private Manipulator manipulator = new Manipulator();
  private Drive drive = new Drive();
  private Timer autotime = PhotonVisionConstants.autotime;

  public Basic() {
    autotime.start();
    drive.zeroGyro();
    autotime.reset();
  }

  public void run() {
    // Equivalent Java code for subsystems::Drive::getInstance().gyro_drive(0.25, 0.0);
    if (autotime.get() < 2.5) {
      manipulator.shoot(1);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (autotime.get() < 3) {
      manipulator.intake(1);
      manipulator.shoot(0.5);
    } else if (autotime.get() < 5) {
      drive.gyroDrive(0.3, 0.0);
    } else {
      drive.gyroDrive(0.0, 0.0);
    }
  }
}
