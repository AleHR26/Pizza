// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.Util;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class MultiNote extends Command {
  private double[] st = {0.0}; // start time, wrapped in an array

  public MultiNote() {
    st[0] = System.currentTimeMillis() / 1000.0; // Get current time in seconds
    SmartDashboard.putNumber("start time", st[0]);
  }

  public void run() {
    // Equivalent of 'using namespace subsystems;' in C++
    // Importing specific classes instead of the entire package
    // to avoid naming conflicts.
    Manipulator manipulator = Manipulator.getInstance();
    Drive drive = Drive.getInstance();

    SmartDashboard.putNumber("time", System.currentTimeMillis() / 1000.0);

    if (Util.wait(st, 2.0)) {
      // Lower arm
      manipulator.shoot(0.5);
      manipulator.armToPos(0.545);
    } else if (Util.wait(st, 4.0)) {
      // Shoot
      manipulator.intake(1.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
    } else if (Util.wait(st, 5.5)) {
      // Drive, intake
      if (manipulator.getNoteSensor()) {
        manipulator.intake(0.375);
      } else {
        manipulator.intake(0.0);
      }
      manipulator.shoot(0.0);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(0.375, 0.0);
    } else if (Util.wait(st, 7.0)) {
      // Drive back
      manipulator.intake(0.0);
      manipulator.shoot(0.5);
      manipulator.armToPos(Manipulator.kARM_FENDER_POS);
      drive.gyroDrive(-0.5, 0.0);
    } else if (Util.wait(st, 9.0)) {
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
