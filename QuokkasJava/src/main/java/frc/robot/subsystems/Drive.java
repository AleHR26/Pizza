package frc.robot.subsystems;

import static frc.robot.Constants.AutoConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainOdometry;

public class Drive extends SubsystemBase {

  public static final double kTrackWidthMeters = 0.65;
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackWidthMeters);

  PIDController xPIDController;
  PIDController yawPIDController;
  DifferentialDriveWheelSpeeds wheelSpeeds;

  public static final Voltage Volts = BaseUnits.Voltage;

  // Crates motor objects
  CANSparkMax L1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax L2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax R1 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax R2 = new CANSparkMax(4, MotorType.kBrushless);
  RelativeEncoder leftEncoder = L1.getEncoder();
  RelativeEncoder RightEncoder = R1.getEncoder();

  DifferentialDriveOdometry m_odometry;
  public DifferentialDrive m_drivetrain;

  public static final AHRS navx = new AHRS(SPI.Port.kMXP);

  // Singleton pattern
  public Drive() {
    // Controls current to motors
    L1.setIdleMode(IdleMode.kBrake);
    L2.setIdleMode(IdleMode.kBrake);
    R1.setIdleMode(IdleMode.kBrake);
    R2.setIdleMode(IdleMode.kBrake);
    L1.setSmartCurrentLimit(40);
    L2.setSmartCurrentLimit(40);
    R1.setSmartCurrentLimit(40);
    R2.setSmartCurrentLimit(40);

    // L2.setInverted(true);
    // R2.setInverted(false);

    // L1.follow(L2);
    // R1.follow(R2);

    //m_drivetrain = new DifferentialDrive(L1, R1);

    leftEncoder.setPosition(0);
    RightEncoder.setPosition(0);

    RightEncoder.setPositionConversionFactor(DrivetrainOdometry.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DrivetrainOdometry.kLinearDistanceConversionFactor);
    RightEncoder.setVelocityConversionFactor(
        DrivetrainOdometry.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(
        DrivetrainOdometry.kLinearDistanceConversionFactor / 60);

    navx.reset();
    resetEncoders();

    // Creates differential drive
   // m_odometry =
      //  new DifferentialDriveOdometry(
          //  navx.getRotation2d(), leftEncoder.getPosition(), RightEncoder.getPosition());

    xPIDController = new PIDController(kPXController, kIXController, kDXController);
    yawPIDController = new PIDController(kPYawController, KIYawController, kDYawController);
  }

  public void driveRobot(ChassisSpeeds speeds) {
    m_drivetrain.feed();

    xPIDController.setSetpoint(speeds.vxMetersPerSecond);
    yawPIDController.setSetpoint(speeds.omegaRadiansPerSecond);

    m_drivetrain.arcadeDrive(
        xPIDController.calculate(getChassisSpeeds().vxMetersPerSecond),
        yawPIDController.calculate(getChassisSpeeds().omegaRadiansPerSecond));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), RightEncoder.getVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        navx.getRotation2d(), leftEncoder.getPosition(), RightEncoder.getPosition(), pose);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setBreakMode() {
    L1.setIdleMode(IdleMode.kBrake);
    L2.setIdleMode(IdleMode.kBrake);
    R1.setIdleMode(IdleMode.kBrake);
    R2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    L1.setIdleMode(IdleMode.kCoast);
    L2.setIdleMode(IdleMode.kCoast);
    R1.setIdleMode(IdleMode.kCoast);
    R2.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    RightEncoder.setPosition(0);
  }

 
  public double getRightEncoderPosition() {
    return RightEncoder.getPosition();
  }

  public double getleftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getEncodersPosition(){
   double position = (RightEncoder.getPosition() +leftEncoder.getPosition())/2;
   return position;

  }

  public double getRightEncoderVelocity() {
    return RightEncoder.getVelocity();
  }

  public double getleftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  private static class DriveHolder {
    private static final Drive INSTANCE = new Drive();
  }

  public static Drive getInstance() {
    return DriveHolder.INSTANCE;
  }

  public double getGyroAngle() {
    return navx.getYaw();
  }

  public void zeroGyro() {
    navx.reset();
  }

  // Calculates power to the motors
  public void move(double power, double steering) {
    double lPower = power - steering;
    double rPower = power + steering;

    L1.set(lPower);
    L2.set(lPower);

    R1.set(rPower);
    R2.set(rPower);
    SmartDashboard.putNumber("lpower", lPower);
    SmartDashboard.putNumber("rpower", rPower);
    
  }

  // Drive with the gyro
  public void gyroDrive(double maxSpeed, double heading) {
    double Kp = 0.008;
    double error = heading - navx.getAngle();
    double steering = Kp * error;
    move(maxSpeed, steering);
    SmartDashboard.putNumber("steering", steering);
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("heading", heading);
  }

    public void setEncoders(double maxSpeed, double position) {
      double kp = 0.01;
      double error = position - (RightEncoder.getPosition() +leftEncoder.getPosition())/2;
      double distance= kp*error;
      move(maxSpeed, distance);
   SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("position", position);
  }


  public double getTurnRate() {
    return -navx.getRate();
  }

  public static double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  private final SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                L1.setVoltage(volts.in(Volts));
                L2.setVoltage(volts.in(Volts));
                R1.setVoltage(volts.in(Volts));
                L2.setVoltage(volts.in(Volts));
              },
              null,
              this));

  @Override
  public void periodic() {
    m_odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), RightEncoder.getPosition());

    SmartDashboard.putNumber("Left encoder value meters", getleftEncoderPosition());
    SmartDashboard.putNumber("Right encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }
}
