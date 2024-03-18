// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class MultiNote {
  private Manipulator manipulator = new Manipulator();
  private Drive drive = new Drive();
  private Timer autotime = PhotonVisionConstants.autotime;

  public MultiNote() {
    autotime.start();
    drive.zeroGyro();
    autotime.reset();
  }

  public void run() {

    
    if (autotime.get() < 2.0) {
      // Lower arm
      manipulator.shoot(1);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (autotime.get() < 4.0) {
      // Shoot
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (autotime.get() < 5.5) {
      // Drive, intake
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.375);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
      drive.gyroDrive(0.375, 0.0);

    } else if (autotime.get() < 6.5) {
      drive.gyroDrive(0.0, 0.0);

    } else if (autotime.get() < 7.0) {
      // Drive back
      manipulator.intake(0.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(-0.45, 0.0);
    } else if (autotime.get() < 10) {
      // Drive back
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(0.0, 0.0);
    } else {
      // Finally,
      drive.move(0.0, 0.0); // Stop
      manipulator.intake(0.0);
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    }
  }
}
