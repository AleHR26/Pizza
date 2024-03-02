package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private static Manipulator instance;

    private DigitalInput input = new DigitalInput(1);
    private DutyCycleEncoder armEnc = new DutyCycleEncoder(input);
    private CANSparkMax armMotorLeft = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax armMotorRight = new CANSparkMax(6, MotorType.kBrushless);

    private CANSparkMax shooterMotorA = new CANSparkMax(7, MotorType.kBrushless);
    private CANSparkMax shooterMotorB = new CANSparkMax(8, MotorType.kBrushless);

    private DigitalInput noteSensor = new DigitalInput(0);
    private CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);

    public static final double kARM_FLOOR_POS = 0.584;  // intaking
    public static final double kARM_FENDER_POS = 0.53;  // close shot
    public static final double kARM_START_POS = 0.376;  // start config
    public static final double kARM_AMP_POS = 0.325;    // amp scoring

    private final double Kp = -15.0;
    private final double maxPower = 0.1;
    

    private Manipulator() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        armMotorLeft.setIdleMode(IdleMode.kCoast);
        armMotorLeft.setOpenLoopRampRate(0.25);
        armMotorRight.setIdleMode(IdleMode.kCoast);
        armMotorRight.setOpenLoopRampRate(0.25);
        armMotorLeft.follow(armMotorRight, true);

    }

    public static Manipulator getInstance() {
        instance = new Manipulator();
        return instance;
    }

    public double getArmEnc() {
        return armEnc.getAbsolutePosition();
    }

  public void armToPos(double pos) {
        double error = pos - armEnc.getAbsolutePosition();
        double power = Kp * error;
        moveArm(power);
    }

    public void moveArm(double power) {
        // Limiting power
        if (power > maxPower) {
            power = maxPower;
        } else if (power < -maxPower) {
            power = -maxPower;
        }

        armMotorLeft.set(-power);
        armMotorRight.set(-power); // Motors are pointing in opposite directions
    }

    public void intake(double power) {
        intakeMotor.set(power);
    }

    public void shoot(double power) {
        shooterMotorA.set(-power);
        shooterMotorB.set(-power); // TODO: Check polarities
    }

    public boolean getNoteSensor() {
        return noteSensor.get();
    }
}