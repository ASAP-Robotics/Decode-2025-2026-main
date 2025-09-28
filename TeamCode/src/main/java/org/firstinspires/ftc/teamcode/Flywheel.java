package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Flywheel {
  private final DcMotorEx flywheel;
  private static final double G = 9.81;
  private static boolean flywheel_isEnabled = false; ///< if the flywheel is enabled

  private static double flywheel_distance = 0; ///< the distance (inches) to the target

  // ---- set these to match your robot ----
  private static final double ANGLE_DEG = 45.0; // shooter pitch
  private static final double DELTA_H_IN = 24.0; // goalHeight - releaseHeight (inches)
  private static final double WHEEL_DIAM_IN = 3.78; // flywheel diameter (inches)

  // ---- friction/slip lumped into one factor (0<eff<=1). Start ~0.90 and tune. ----
  private static final double EFFICIENCY = 0.90;

  public Flywheel(DcMotorEx motor) {
    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // brake if zero power (motor stoped)
    this.flywheel = motor;
    this.flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
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
      startMotor(); // set the motor to the correct speed
    } else {
      stopMotor(); // stop the flywheel
    }
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
