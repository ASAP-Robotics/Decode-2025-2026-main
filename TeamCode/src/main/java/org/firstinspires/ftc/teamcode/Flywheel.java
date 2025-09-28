package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Flywheel {
  private final DcMotorEx flywheel;
  private static final double G = 9.81;
  private static boolean flywheel_isEnabled = false; // /< if the flywheel is enabled
  private static boolean flywheel_isActive = true; // /< if the flywheel is active (as opposed to idling)
  private static double flywheel_idleSpeed; // /< the speed (RPM) of the flywheel when idle
  private static double flywheel_distance = 0; // /< the distance (inches) to the target

  // ---- set these to match your robot ----
  private static final double ANGLE_DEG = 45.0; // shooter pitch
  private static final double DELTA_H_IN = 24.0; // goalHeight - releaseHeight (inches)
  private static final double WHEEL_DIAM_IN = 3.78; // flywheel diameter (inches)

  // ---- friction/slip lumped into one factor (0<eff<=1). Start ~0.90 and tune. ----
  private static final double EFFICIENCY = 0.90;

  /**
   * @brief makes an object of the Flywheel class with and idle speed of 500 RPM
   * @param motor the motor used for the flywheel
   */
  public Flywheel(DcMotorEx motor) {
    this(motor, 500); // make a Flywheel with an idle speed of 500 RPM
  }

  /**
   * @brief makes an object of the Flywheel class
   * @param motor the motor used for the flywheel
   * @param idleSpeed the speed of the flywheel when idling (RPM)
   */
  public Flywheel(DcMotorEx motor, double idleSpeed) {
    motor.setZeroPowerBehavior(
            DcMotorEx.ZeroPowerBehavior.BRAKE); // brake if zero power (motor stoped)
    this.flywheel = motor;
    this.flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
    flywheel_idleSpeed = idleSpeed; // set the speed of the flywheel at idle
  }

  /**
   * @brief sets if the flywheel is enabled
   * @param isEnabled if the flywheel will be enabled (true = enabled, false = not enabled)
   * @return the previous enabled / disabled state of the flywheel
   */
  public boolean setEnabled(boolean isEnabled) {
    boolean toReturn = flywheel_isEnabled; // get the old enabled state
    flywheel_isEnabled = isEnabled; // set the new enabled state
    update(); // apply any changes
    return toReturn; // return the old enabled state
  }

  /**
   * @brief enables the flywheel
   * @return the previous enabled / disabled state of the flywheel
   */
  public boolean enable() {
    return setEnabled(true);
  }

  /**
   * @brief disables the flywheel
   * @return the previous enabled / disabled state of the flywheel
   */
  public boolean disable() {
    return setEnabled(false);
  }

  /**
   * @brief sets if the flywheel is active (as opposed to idling)
   * @param isActive The new activity state, true if active, false if idling
   * @return the old activation state, true if active, false if idling
   */
  public boolean setActive(boolean isActive) {
    boolean toReturn = flywheel_isActive; // store the old activation state
    flywheel_isActive = isActive; // update if the flywheel is active or idling
    update(); // apply any changes
    return toReturn; // return the old activation state
  }

  /**
   * @brief activate the flywheel (spin it up to full speed from idle speed)
   * @return the old activation state of the flywheel (true if active, false if idle)
   */
  public boolean activate() {
    return setActive(true);
  }

  /**
   * @brief idles the flywheel (lower it to idle speed from full speed)
   * @return the old activation state of the flywheel (true if active, false if idle)
   */
  public boolean idle() {
    return setActive(false);
  }

  public double setIdleSpeed(double idleSpeed) {
    double toReturn = flywheel_idleSpeed; // store the old idle speed
    flywheel_idleSpeed = idleSpeed; // set the new flywheel idle speed
    update(); // apply any changes
    return toReturn; // return the old idle speed
  }

  /**
   * returns the speed of the flywheel when idle
   * @return the idle speed of the flywheel, in RPM
   */
  public double getIdleSpeed() {
    return flywheel_idleSpeed; // return the idle speed
  }

  /**
   * @brief sets the distance to the target
   * @param distInches the new distance to the target, in inches
   * @return the old distance to the target, in inches
   */
  public double setTargetDistance(double distInches) {
    double toReturn = flywheel_distance; // store the old distance target
    flywheel_distance = distInches; // set the new distance target
    update(); // apply any changes
    return toReturn; // return the old distance target
  }

  /**
   * @brief returns the distance to the target being used
   * @return the distance to the target, in inches
   */
  public double getTargetDistance() {
    return flywheel_distance; // return the distance target
  }

  /**
   * @brief updates the flywheel, setting the motor to the correct speed
   */
  private void update() {
    if (flywheel_isEnabled) {
      if (flywheel_isActive) {
        startMotor(); // set the motor to the correct speed
      } else {
        idleMotor(); // set the motor to idling speeds
      }
    } else {
      stopMotor(); // stop the flywheel
    }
  }

  /**
   * @brief sets the motor to the idle speed
   * @note use setIdleSpeed() to set the idle speed
   */
  private void idleMotor() {
    double ticksPerRev = flywheel.getMotorType().getTicksPerRev();
    double ticksPerSec = (flywheel_idleSpeed / 60.0) * ticksPerRev;
    flywheel.setVelocity(ticksPerSec); // set the speed using the built-in PID controller
  }

  /**
   * @brief sets the motor to the correct speed to hit the target
   * @note use `setTargetDistance()` to set the distance from the target
   */
  private void startMotor() {
    double rpm = rpmForDistance(flywheel_distance);
    double ticksPerRev = flywheel.getMotorType().getTicksPerRev();
    double ticksPerSec = (rpm / 60.0) * ticksPerRev;
    flywheel.setVelocity(ticksPerSec); // built-in velocity PID
  }

  /**
   * @brief stops the flywheel
   */
  private void stopMotor() {
    flywheel.setPower(0);
  }

  /**
   * @brief finds the correct flywheel speed to hit a target a specified distance away
   * @param Rin the target distance in inches
   * @return the correct flywheel RPM to hit the target
   * @note Core math: distance (in) → wheel RPM, including efficiency loss
   */
  private double rpmForDistance(double Rin) {
    // convert to meters
    double Rm = Rin * 0.0254;
    double deltaHm = DELTA_H_IN * 0.0254;
    double wheelDiamM = WHEEL_DIAM_IN * 0.0254;

    double theta = Math.toRadians(ANGLE_DEG);
    double cos = Math.cos(theta), tan = Math.tan(theta);
    double denom = 2.0 * cos * cos * (Rm * tan - deltaHm);
    if (denom <= 0) throw new IllegalArgumentException("Unreachable shot at this distance.");

    double vExit = Math.sqrt(G * Rm * Rm / denom); // ideal exit speed (m/s)

    // ---- Apply efficiency: wheel surface speed must be higher than exit speed. ----
    double wheelSurfaceSpeed = vExit / EFFICIENCY;

    return (60.0 * wheelSurfaceSpeed) / (Math.PI * wheelDiamM); // RPM
  }
}
