// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.Timer;

public class Util {
  /* Note that step_start is passed by reference and incremented by `duration` seconds. */
  public static boolean wait(double[] stepStart, double duration) {
    boolean result = Timer.getFPGATimestamp() < stepStart[0] + duration;
    stepStart[0] += duration; // Increment step_start by duration
    return result;
  }
}
