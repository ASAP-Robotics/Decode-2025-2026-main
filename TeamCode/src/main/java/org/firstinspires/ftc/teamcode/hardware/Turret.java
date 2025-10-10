package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret extends Flywheel {
  private final DcMotorEx rotator;
  private final Servo hoodServo;
  private double targetHorizontalAngleDegrees; // target angle for side-to-side turret movement
  private double targetVerticalAngleDegrees; // target angle for up-and-down turret movement

  public Turret(DcMotorEx flywheelMotor, DcMotorEx rotator, Servo hoodServo) {
    super(flywheelMotor);
    this.rotator = rotator;
    this.hoodServo = hoodServo;
    this.rotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    this.rotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  }

  public Turret(
      DcMotorEx flywheelMotor,
      DcMotorEx rotator,
      Servo hoodServo,
      double idleSpeed,
      double shotTimeSeconds) {
    super(flywheelMotor, idleSpeed, shotTimeSeconds); // create Flywheel
    this.rotator = rotator;
    this.hoodServo = hoodServo;
    this.rotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    this.rotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  }

  @Override
  public void update() {
    super.update();
    hoodServo.setPosition(targetVerticalAngleDegrees); // this might need updating
    double ticksPerDegree = rotator.getMotorType().getTicksPerRev() / 360;
    double motorDegrees = turretDegreesToMotorDegrees(targetHorizontalAngleDegrees);
    rotator.setTargetPosition((int) (motorDegrees * ticksPerDegree));
  }

  /**
   * @brief sets if the turret is enabled
   * @param isEnabled if the turret will be enabled (true = enabled, false = not enabled)
   * @return the previous enabled / disabled state of the turret
   */
  @Override
  public boolean setEnabled(boolean isEnabled) {
    rotator.setPower(isEnabled ? 1 : 0); // 0% power if disabled, 100% power if enabled
    return super.setEnabled(isEnabled);
  }

  /**
   * @brief sets the side-to-side angle of the turret in degrees
   * @param degrees the number of degrees from straight to move the turret
   * @return the old horizontal angle of the turret
   */
  public double setHorizontalAngle(double degrees) {
    double toReturn = targetHorizontalAngleDegrees;
    targetHorizontalAngleDegrees = degrees;
    update();
    return toReturn;
  }

  /**
   * @brief returns the target side-to-side angle of the turret
   * @return the target horizontal angle of the turret
   */
  public double getTargetHorizontalAngleDegrees() {
    return targetHorizontalAngleDegrees;
  }

  /**
   * @brief sets the up-and-down angle of the turret in degrees
   * @param degrees the number of degrees from straight to move the turret
   * @return the old vertical angle of the turret
   */
  public double setVerticalAngle(double degrees) {
    double toReturn = targetVerticalAngleDegrees;
    targetVerticalAngleDegrees = degrees;
    update();
    return toReturn;
  }

  /**
   * @brief returns the target up-and-down angle of the turret
   * @return the target vertical angle of the turret
   */
  public double getTargetVerticalAngleDegrees() {
    return targetVerticalAngleDegrees;
  }

  /**
   * @brief finds the number of degrees the motor needs to turn for the turret to turn some amount
   * @param turretDegrees the number of degrees the turret should move by
   * @return the number of degrees the motor should turn to get the turret to move correctly
   */
  private double turretDegreesToMotorDegrees(double turretDegrees) {
    double MOTOR_GEAR_TEETH = 24; // number of teeth on the gear attached to the motor
    double TURRET_GEAR_TEETH = 120; // number of teeth on the gear attached to the turret
    return turretDegrees * (TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH);
  }
}
