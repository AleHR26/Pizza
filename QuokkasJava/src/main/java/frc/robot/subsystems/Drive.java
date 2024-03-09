package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  // Crates motor objects
  private CANSparkMax L1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax L2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax R1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax R2 = new CANSparkMax(4, MotorType.kBrushless);

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  // Singleton pattern
  public Drive() {
    // Controls current to motors
    L1.setIdleMode(IdleMode.kBrake);
    L2.setIdleMode(IdleMode.kBrake);
    R1.setIdleMode(IdleMode.kBrake);
    R2.setIdleMode(IdleMode.kBrake);
    L1.setSmartCurrentLimit(60);
    L2.setSmartCurrentLimit(60);
    R1.setSmartCurrentLimit(60);
    R2.setSmartCurrentLimit(60);
    // Creates differential drive
  }

  private static class DriveHolder {
    private static final Drive INSTANCE = new Drive();
  }

  public static Drive getInstance() {
    return DriveHolder.INSTANCE;
  }

  // Resets gyro
  public void zeroGyro() {
    navx.zeroYaw();
  }

  public double getGyroAngle() {
    return navx.getAngle();
  }

  // Calculates power to the motors
  public void move(double power, double steering) {
    double lPower = power - steering;
    double rPower = power + steering;

    L1.set(lPower);
    L2.set(lPower);

    R1.set(rPower);
    R2.set(rPower);
  }

  // Drive with the gyro
  public void gyroDrive(double maxSpeed, double heading) {
    double Kp = 0.015;
    double error = heading - navx.getAngle();
    double steering = Kp * error;
    move(maxSpeed, steering);
  }
}
