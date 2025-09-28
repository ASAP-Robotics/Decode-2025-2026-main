package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Flywheel {
  private final DcMotorEx flywheel;
  private static final double G = 9.81;

  // ---- set these to match your robot ----
  private static final double ANGLE_DEG = 45.0; // shooter pitch
  private static final double DELTA_H_IN = 24.0; // goalHeight - releaseHeight (inches)
  private static final double WHEEL_DIAM_IN = 3.78; // flywheel diameter (inches)

  // ---- friction/slip lumped into one factor (0<eff<=1). Start ~0.90 and tune. ----
  private static final double EFFICIENCY = 0.90;

  public Flywheel(DcMotorEx motor) {
    this.flywheel = motor;
    this.flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  /** Start motor at correct velocity for given distance (inches). */
  public void startMotor(double distInches) {
    double rpm = rpmForDistance(distInches);
    double ticksPerRev = flywheel.getMotorType().getTicksPerRev();
    double ticksPerSec = (rpm / 60.0) * ticksPerRev;
    flywheel.setVelocity(ticksPerSec); // built-in velocity PID
  }

  /** Stop the flywheel. */
  public void stopMotor() {
    flywheel.setPower(0);
  }

  /** Core math: distance (in) → wheel RPM, including efficiency loss. */
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
