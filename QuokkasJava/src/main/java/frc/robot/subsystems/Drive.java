package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drive extends SubsystemBase {
  // Crates motor objects
  private CANSparkMax L1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax L2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax R1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax R2 = new CANSparkMax(4, MotorType.kBrushless);

  RelativeEncoder leftEncoder = L1.getEncoder(); // REVISAR
  RelativeEncoder rightEncoder = R1.getEncoder(); // REVISAR
  DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition()); // REVISAR

  public final static AHRS navX = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry m_Odometry;

  DifferentialDrive differentialDrive = new DifferentialDrive(L1, R1); // REVISAR

  // Singleton pattern
  public Drive() {
    // Controls current to motors
    setBreakMode();
    L1.setSmartCurrentLimit(60);
    L2.setSmartCurrentLimit(60);
    R1.setSmartCurrentLimit(60);
    R2.setSmartCurrentLimit(60);
    // Creates differential drive

    rightEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);

    zeroGyro(); //navX.reset();
    System.err.println("Is navX calibrating? = "+ navX.isCalibrating()); //navX.calibrate();
    resetEncoders();

    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    m_Odometry.resetPosition(navX.getRotation2d(), wheelPositions, new Pose2d());
  }

  public void setBreakMode(){
    L1.setIdleMode(IdleMode.kBrake);
    L2.setIdleMode(IdleMode.kBrake);
    R1.setIdleMode(IdleMode.kBrake);
    R2.setIdleMode(IdleMode.kBrake);
  }
  public void setCoastMode(){
    L1.setIdleMode(IdleMode.kCoast);
    L2.setIdleMode(IdleMode.kCoast);
    R1.setIdleMode(IdleMode.kCoast);
    R2.setIdleMode(IdleMode.kCoast);
  }
  public void resetEncoders(){ // REVISAR
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
  private static class DriveHolder {
    private static final Drive INSTANCE = new Drive();
  }
  
  public static Drive getInstance() {
    return DriveHolder.INSTANCE;
  }


  public double getLeftEncoderPosition(){
    return leftEncoder.getPosition(); //Negate IF results in smartdashboard are negative.
  }
  public double getRightEncoderPosition(){
    return rightEncoder.getPosition(); //Negate IF results in smartdashboard are negative.
  }
  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity(); //Negate IF results in smartdashboard are negative.
  }
  public double getRightEncoderVelocity(){
    return rightEncoder.getVelocity(); //Negate IF results in smartdashboard are negative.
  }
  public double getTurnRate(){
    return -navX.getRate();
  }
  public static double getHeading() {
    return navX.getRotation2d().getDegrees();
  }
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_Odometry.resetPosition(navX.getRotation2d(), wheelPositions, pose);
  }
  public DifferentialDriveWheelSpeeds getwWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  public void driveVolts(double leftVolts, double rightVolts){ // REVISAR
    L1.setVoltage(leftVolts);
    L2.setVoltage(leftVolts);
    R1.setVoltage(rightVolts);
    R1.setVoltage(rightVolts);
    differentialDrive.feed();
  }
  public double getAverageEncoderDistance(){
    return ((getLeftEncoderPosition() + getRightEncoderPosition())/2.0);
  }
  public RelativeEncoder getLefEncoder(){
    return leftEncoder;
  }
  public RelativeEncoder getRightEncoder(){
    return rightEncoder;
  }
  public void setMaxOutput(double maxOutput){ // REVISAR
    differentialDrive.setMaxOutput(maxOutput);
  }
  public static void zeroHeading(){
    System.err.println("Is navX calibrating? = "+ navX.isCalibrating()); //navX.calibrate();
    navX.zeroYaw();
  }


  @Override
  public void periodic() {
    m_Odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    wheelPositions = new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("LEFT encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("RIGHT encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }


  // Resets gyro
  public void zeroGyro() {
    navX.zeroYaw();
  }

  public double getGyroAngle() {
    return navX.getAngle();
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
    double error = heading - navX.getAngle();
    double steering = Kp * error;
    move(maxSpeed, steering);
  }
}
