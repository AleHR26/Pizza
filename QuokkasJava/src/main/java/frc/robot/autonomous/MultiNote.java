// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class MultiNote {
  // start time, wrapped in an array
  private Manipulator manipulator = new Manipulator();
  private Drive drive = new Drive();
  double autonomousStartTime;

  public MultiNote() {
    autonomousStartTime = Timer.getFPGATimestamp(); // Get current time in seconds
    SmartDashboard.putNumber("start time", autonomousStartTime);
  }

  public void run() {
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    // Equivalent of 'using namespace subsystems;' in C++
    // Importing specific classes instead of the entire package
    // to avoid naming conflicts.

    if (timeElapsed < 2.0) {
      // Lower arm
      manipulator.shoot(0.2);
      //manipulator.armToPos(0.235);
    } /*else if (timeElapsed < 4.0) {
          // Shoot
          manipulator.intake(1.0);
          manipulator.shoot(0.5);
          manipulator.armToPos(Manipulator.kARM_FENDER_POS);
        } else if (timeElapsed < 5.5) {
          // Drive, intake
          if (manipulator.getNoteSensor()) {
            manipulator.intake(0.375);
          } else {
            manipulator.intake(0.0);
          }
          manipulator.shoot(0.0);
          manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
          drive.gyroDrive(0.375, 0.0);
        } else if (timeElapsed < 7.0) {
          // Drive back
          manipulator.intake(0.0);
          manipulator.shoot(0.5);
          manipulator.armToPos(Manipulator.kARM_FENDER_POS);
          drive.gyroDrive(-0.5, 0.0);
        } else if (timeElapsed < 9.0) {
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
      /* */
  }
}
