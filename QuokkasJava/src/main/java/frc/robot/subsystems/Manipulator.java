// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  private static Manipulator instance;

  private Encoder armEnc = new Encoder(1, 2);
  private CANSparkMax armMotorLeft = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax armMotorRight = new CANSparkMax(6, MotorType.kBrushless);

  private CANSparkMax shooterMotorA = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax shooterMotorB = new CANSparkMax(8, MotorType.kBrushless);

  private DigitalInput noteSensor = new DigitalInput(0);
  private CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);

  public static final double kARM_FLOOR_POS = 0.584; // intaking
  public static final double kARM_FENDER_POS = 0.53; // close shot
  public static final double kARM_START_POS = 0.376; // start config
  public static final double kARM_AMP_POS = 0.325; // amp scoring

  private final PIDController armController = new PIDController(15.0, 0, 0);

  private Manipulator() {
    armMotorLeft.follow(armMotorRight);
    shooterMotorB.follow(shooterMotorA);

    intakeMotor.setSmartCurrentLimit(60);
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
    if (instance == null) {
      instance = new Manipulator();
    }
    return instance;
  }

  public double getArmEnc() {
    return armEnc.get();
  }

  /* See Manipulator::kARM_FLOOR_POS etc. */
  public void armToPos(double pos) {
    double power = -armController.calculate(armEnc.get(), pos);
    moveArm(power);
  }

  public void moveArm(double power) {
    final double maxPower = 0.5;

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
