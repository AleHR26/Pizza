// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Manipulator;

public class MultiNote extends Command {
  private double st = 0; // start time, wrapped in an array
  private Manipulator manipulator = new Manipulator();
  private Drive drive = new Drive();
  private Timer t = new Timer();

  @Override
  public void initialize() {
    t.start();
  }

  public MultiNote() {
    st = t.get();
    SmartDashboard.putNumber("start time", st);
  }

  public void run() {
    // Equivalent of 'using namespace subsystems;' in C++
    // Importing specific classes instead of the entire package
    // to avoid naming conflicts.

    if (st < 2) {
      // Lower arm
      manipulator.shoot(0.5);
      // manipulator.armToPos(0.275);
    }  /*else if (st < 4) {
          // Shoot
          manipulator.intake(1.0);
          manipulator.shoot(0.5);
          manipulator.armToPos(Manipulator.kARM_FENDER_POS);
        } else if (st < 5.5) {
          // Drive, intake
          if (manipulator.getNoteSensor()) {
            manipulator.intake(0.375);
          } else {
            manipulator.intake(0.0);
          }
          manipulator.shoot(0.0);
          manipulator.armToPos(Manipulator.kARM_FLOOR_POS);
          drive.gyroDrive(0.375, 0.0);
        } else if (st < 7.0) {
          // Drive back
          manipulator.intake(0.0);
          manipulator.shoot(0.5);
          manipulator.armToPos(Manipulator.kARM_FENDER_POS);
          drive.gyroDrive(-0.5, 0.0);
        } else if (st < 9.0) {
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

  @Override
  public void end(boolean interrupted) {
    drive.move(0.0, 0.0); // Stop
    manipulator.intake(0.0);
    manipulator.shoot(0.0);
    manipulator.armToPos(Manipulator.kARM_FENDER_POS);
  }



}
