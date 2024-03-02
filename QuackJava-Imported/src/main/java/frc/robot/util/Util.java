package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class Util {
    /* Note that step_start is passed by reference and incremented by `duration` seconds. */
    public static boolean wait(double[] stepStart, double duration) {
         boolean result = Timer.getFPGATimestamp() < stepStart[0] + duration;
        stepStart[0] += duration; // Increment step_start by duration
        return result;
    }
}

