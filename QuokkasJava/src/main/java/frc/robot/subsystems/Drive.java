package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private CANSparkMax L1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax L2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax R1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax R2 = new CANSparkMax(4, MotorType.kBrushless);
  DifferentialDrive drivetrain;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  // Singleton pattern
  public Drive() {
    L2.follow(L1);
    R2.follow(R1);

    L1.setSmartCurrentLimit(60);
    L2.setSmartCurrentLimit(60);
    R1.setSmartCurrentLimit(60);
    R2.setSmartCurrentLimit(60);

    L1.setInverted(true);
    L2.setInverted(true);
    R1.setInverted(false);
    R2.setInverted(false);

    drivetrain = new DifferentialDrive(L1, R1);
  }

  private static class DriveHolder {
    private static final Drive INSTANCE = new Drive();
  }

  public static Drive getInstance() {
    return DriveHolder.INSTANCE;
  }

  public void zeroGyro() {
    navx.zeroYaw();
  }

  public void move(double power, double steering) {
    double lPower = power - steering;
    double rPower = power + steering;

    L1.set(lPower);
    L2.set(lPower);

    R1.set(rPower);
    R2.set(rPower);
  }

  public void gyroDrive(double maxSpeed, double heading) {
    double Kp = 0.015;
    double error = heading - navx.getAngle();
    double steering = Kp * error;
    move(maxSpeed, steering);
  }
}
