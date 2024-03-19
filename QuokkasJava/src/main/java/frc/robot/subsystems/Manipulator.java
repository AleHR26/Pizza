// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
  private static Manipulator instance;

  private DutyCycleEncoder armEnc = new DutyCycleEncoder(1);
  private CANSparkMax armMotorLeft = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax armMotorRight = new CANSparkMax(6, MotorType.kBrushless);

  private CANSparkMax shooterMotorA = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax shooterMotorB = new CANSparkMax(8, MotorType.kBrushless);

  RelativeEncoder aEncoder = shooterMotorA.getEncoder();
  RelativeEncoder bEncoder = shooterMotorB.getEncoder();

  public static Rev2mDistanceSensor noteSensor = new Rev2mDistanceSensor(Port.kOnboard);
  private CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);

  public static final double kARM_FLOOR_POS = 0.0175; // intaking 0.181
  public static final double kARM_FENDER_POS = 0.064; // close shot 0.235
  public static final double kARM_START_POS = 0.23; // start config 0.376
  public static final double kARM_AMP_POS = 0.2665; // amp scoring 0.43
  private final double Kp = -15.0;
  private final double kd = -9.0;
  private final double maxPower = 0.5;
  double lasterror = 0;

  public Manipulator() {
    armMotorLeft.follow(armMotorRight, true);
    // shooterMotorB.follow(shooterMotorA);

    intakeMotor.setSmartCurrentLimit(20);
    armMotorRight.setSmartCurrentLimit(40);
    armMotorLeft.setSmartCurrentLimit(40);
    shooterMotorA.setSmartCurrentLimit(40);
    shooterMotorB.setSmartCurrentLimit(40);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setOpenLoopRampRate(0.25);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armMotorRight.setOpenLoopRampRate(0.25);

    shooterMotorA.setIdleMode(IdleMode.kBrake);
    shooterMotorB.setIdleMode(IdleMode.kBrake);

    noteSensor.setAutomaticMode(true);

    aEncoder.setPosition(0); 
    bEncoder.setPosition(0); 

    resetShooters();
  }

  public void resetShooters() {
    aEncoder.setPosition(0);
    bEncoder.setPosition(0);
  }

  public double getaEncoderPosition() {
    return aEncoder.getPosition();
  }

   public double getbEncoderPosition() {
    return bEncoder.getPosition();
  }

  public double getShootersPosition(){
   double position = (aEncoder.getPosition() +bEncoder.getPosition())/2;
   return position;

  }

  public double getShooterAVelocity() {
    return aEncoder.getVelocity();
  }

  public double getShooterBVelocity() {
    return bEncoder.getVelocity();
  }

  public static Manipulator getInstance() {
    instance = new Manipulator();
    return instance;
  }

  public double getArmEnc() {
    return armEnc.getAbsolutePosition();
  }

  /* See Manipulator::kARM_FLOOR_POS etc. */
  public void armToPos(double pos) {

    double error = pos - armEnc.getAbsolutePosition();
    double errorrate = error - lasterror;
    double power = Kp * error + kd * errorrate;
    moveArm(power);
    SmartDashboard.putNumber("ArmPower", power);
    lasterror = error;
  }

  public void moveArm(double power) {

    // Stop from making too much torque
    if (power > maxPower) {
      power = maxPower;
    } else if (power < -maxPower) {
      power = -maxPower;
    }

    armMotorLeft.set(-power);
    armMotorRight.set(-power); // -ve, as motors are pointing in opposite directions
  }

  public void intake(double power) {
    intakeMotor.set(power);
  }

  public void shoot(double power) {
    shooterMotorA.set(-power); //setVoltage(12.0 * rpm / 5676.0 + 12.0 * .0001 * (rpm - getShooterAVelocity()));
    shooterMotorB.set(-power); 
  }

  public double getRange() {
    return noteSensor.GetRange();
  }

  public boolean getNoteSensor() {
    if (noteSensor.getRange() > 4) {
      return true;
    } else {
      return false;
    }
  }
}
