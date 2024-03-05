// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  private static Manipulator instance;

  private DutyCycleEncoder armEnc = new DutyCycleEncoder(1);
  private CANSparkMax armMotorLeft = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax armMotorRight = new CANSparkMax(6, MotorType.kBrushless);

  private CANSparkMax shooterMotorA = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax shooterMotorB = new CANSparkMax(8, MotorType.kBrushless);

  private DigitalInput noteSensor = new DigitalInput(0);
  private CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);

  // TODO: CONFIGURE THE POSITIONS FOR THE ENCODER
  public static final double kARM_FLOOR_POS = 0.17; // intaking (Before: 0.584)
  public static final double kARM_FENDER_POS = 0.222; // close shot (Before: 0.53)
  public static final double kARM_START_POS = 0.376; // start config (Before: 0.376)
  public static final double kARM_AMP_POS = 0.427; // amp scoring (Before: 0.325)

  private Manipulator() {
    armMotorLeft.follow(armMotorRight, true);
    // shooterMotorB.follow(shooterMotorA);

    intakeMotor.setSmartCurrentLimit(20);
    armMotorRight.setSmartCurrentLimit(60);
    armMotorLeft.setSmartCurrentLimit(60);
    shooterMotorA.setSmartCurrentLimit(60);
    shooterMotorB.setSmartCurrentLimit(60);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setOpenLoopRampRate(0.25);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armMotorRight.setOpenLoopRampRate(0.25);
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
    double Kp = -15.0;
    double error = pos - armEnc.getAbsolutePosition();
    double power = Kp * error;
    moveArm(power);
  }

  public void moveArm(double power) {
    final double maxPower = 0.1;

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
    shooterMotorA.set(-power);
    shooterMotorB.set(-power); // TODO: check polarities
  }

  public boolean getNoteSensor() {
    return noteSensor.get();
  }
}
