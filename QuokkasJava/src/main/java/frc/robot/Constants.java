// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

  public static class PhotonVisionConstants {
    // Constants such as camera and target height stored. Change per robot and goal!
    static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
    static final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.4);
    // Angle between horizontal and the camera.
    static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    static final double GOAL_RANGE_METERS = Units.feetToMeters(1.5);
  }

  public static class DrivetrainConstants {
    /**   * How many amps can an individual drivetrain motor use.   */
    static final int DRIVE_CURRENT_LIMIT_A = 60;

    /**   * How many amps the feeder motor can use.   */
    static final int FEEDER_CURRENT_LIMIT_A = 80;

    /**   * Percent output to run the feeder when expelling note   */
    static final double FEEDER_OUT_SPEED = 1.0;

    /**   * Percent output to run the feeder when intaking note   */
    static final double FEEDER_IN_SPEED = -.4;

    /**   * Percent output for amp or drop note, configure based on polycarb bend   */
    static final double FEEDER_AMP_SPEED = .4;

    /**
     *   * How many amps the launcher motor can use.   *   * In our testing we favored the CIM over
     * NEO, if using a NEO lower this to 60
     */
    static final int LAUNCHER_CURRENT_LIMIT_A = 80;

    /**   * Percent output to run the launcher when intaking AND expelling note   */
    static final double LAUNCHER_SPEED = 1.0;

    /**
     *   * Percent output for scoring in amp or dropping note, configure based on polycarb bend .14
     * works well with no bend from our testing
     */
    static final double LAUNCHER_AMP_SPEED = .17;

    /** Percent output for the roller claw */
    static final double CLAW_OUTPUT_POWER = .5;

    /** Percent output to help retain notes in the claw */
    static final double CLAW_STALL_POWER = .1;

    /** Percent output to power the climber */
    static final double CLIMER_OUTPUT_POWER = 1;
  }
}
