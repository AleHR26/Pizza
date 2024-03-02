package frc.robot.autonomous;

import frc.robot.subsystems.Drive;

public class Basic {
    
    public Basic() {
        // Constructor logic (if any)
    }

    public void run() {
        // Equivalent Java code for subsystems::Drive::getInstance().gyro_drive(0.25, 0.0);
        Drive.getInstance().gyroDrive(0.25, 0.0);
    }
}