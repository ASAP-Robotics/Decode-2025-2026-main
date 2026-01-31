/*
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.teamcode.hardware;

import android.util.Pair;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;
import java.util.LinkedList;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;
import org.firstinspires.ftc.teamcode.hardware.sensors.Limelight;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;
import org.jetbrains.annotations.TestOnly;

public class ScoringSystem {
  public enum State {
    UNINITIALISED,
    FULL,
    INTAKING,
    SHOOTING
  }

  private final ActiveIntake intake; // the intake on the robot
  private final Turret turret; // the flywheel on the robot
  private final Spindex spindex; // the spindex on the robot
  private final Limelight limelight; // the limelight camera on the turret
  private final RGBIndicator indicator1; // first indicator light
  private final RGBIndicator indicator2; // second indicator light
  protected State state = State.UNINITIALISED; // the state of the scoring system
  private BallSequence ballSequence = BallSequence.GPP; // the sequence being shot
  public final AllianceColor allianceColor; // the alliance we are on
  private boolean turretAimOverride = false; // if the aim of the turret is overridden
  private double horizontalAngleOverride = 0;
  private boolean tuning = false;
  private double verticalAngleOverride = 60;
  private double rpmOverride = 2000;
  private double distanceOverride = 1;
  private final Pose2D targetPosition; // the position of the target to shoot at
  private Pose2D robotPosition =
      new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0); // the position of the robot
  private boolean clearingIntake = false; // if the intake is being reversed to clear a blockage
  private final Telemetry telemetry;
  private final SimpleTimer fullWait = new SimpleTimer(0.5);
  private final ElapsedTime timeSinceStart = new ElapsedTime();
  private final ElapsedTime loopTime = new ElapsedTime();
  private final LinkedList<Pair<Double, Double>> loopTimes = new LinkedList<>();

  public ScoringSystem(
      ActiveIntake intake,
      Turret turret,
      Spindex spindex,
      Limelight limelight,
      RGBIndicator indicator1,
      RGBIndicator indicator2,
      AllianceColor allianceColor,
      Telemetry telemetry) {
    this.intake = intake;
    this.turret = turret;
    this.spindex = spindex;
    this.limelight = limelight;
    this.indicator1 = indicator1;
    this.indicator2 = indicator2;
    this.allianceColor = allianceColor;
    this.telemetry = telemetry;
    this.targetPosition = this.allianceColor.getTargetLocation();
  }

  /**
   * Initializes the artifact scoring system
   *
   * @param isPreloaded if the spindex is preloaded with balls
   * @param auto if opMode is Auto (as opposed to TeleOp)
   * @note call when OpMode is initialized ("Init" is pressed)
   */
  public void init(boolean isPreloaded, boolean auto) {
    setIndicatorColor(RGBIndicator.Color.VIOLET);
    spindex.init(BallSequence.GPP, isPreloaded, auto);
    turret.init(0);
    turret.setActive(!isPreloaded);
    limelight.init(auto);
  }

  /** To be called repeatedly while the robot is in init */
  public void initLoop() {
    spindex.update();
    turret.update();
  }

  /**
   * Starts scoring systems up
   *
   * @note call when OpMode is started ("Start" is pressed)
   */
  public void start(boolean isPreloaded, boolean search) {
    spindex.start();
    turret.enable(); // let the flywheel spin up
    turret.start();
    limelight.start();
    if (search) limelight.detectSequence();
    state = isPreloaded ? State.FULL : State.INTAKING;
    timeSinceStart.reset();
  }

  /**
   * Stops all powered movement as quickly as possible (think E-stop)
   *
   * @note intended to be called when the "Stop" button is pressed
   */
  public void stop() {
    turret.disable(); // stop the flywheel
    turret.update();
    intake.stop(); // stop the intake
  }

  /**
   * Updates everything to do with the artifact scoring system
   *
   * @note call each loop
   */
  public void update(boolean updateTelemetry) {
    limelight.update();
    ballSequence = limelight.getSequence();
    updateAiming();
    updateSpindex();
    updateIntake();
    intake.update();
    turret.update();
    spindex.update();
    updateIndicators();
    if (updateTelemetry) updateTelemetry();
    double now = timeSinceStart.milliseconds();
    loopTimes.push(new Pair<>(loopTime.milliseconds(), now));
    loopTime.reset();
    while (!loopTimes.isEmpty() && now - loopTimes.getFirst().second > 1) {
      loopTimes.removeFirst();
    }
  }

  /** Updates everything to do with aiming the turret */
  private void updateAiming() {
    if (tuning) {
      turret.tuneShooting(rpmOverride, verticalAngleOverride);

    } else if (turretAimOverride) {
      turret.setHorizontalAngle(horizontalAngleOverride);
      turret.setTargetDistance(distanceOverride);
      return;
    }
    /*
    else if (!limelight.isReadyToNavigate()) {
      turret.setHorizontalAngle(allianceColor.getObeliskAngle());
      return;
    }
     */

    /*
    Pose2D limelightPosition = limelight.getPosition();
    if (limelightPosition != null && turret.isAtTarget()) {
      double targetLimelightHeading =
          robotPosition.getHeading(AngleUnit.DEGREES) + turret.getHorizontalAngleDegrees();
      double limelightHeading = limelightPosition.getHeading(AngleUnit.DEGREES);
      double headingError = targetLimelightHeading - limelightHeading;
      if (Math.abs(headingError) >= 1) turret.changeHorizontalAngleOffsetDegrees(headingError);
    }
     */

    turret.setHorizontalAngle(getRelativeTargetAngle());
    turret.setTargetDistance(getTargetDistance());
  }

  /** Updates everything to do with the spindexer */
  private void updateSpindex() {
    spindex.setSequence(ballSequence);

    switch (spindex.getState()) {
      case INTAKING:
        if (state != State.INTAKING) switchModeToIntaking();
        break;

      case SHOOTING_READY:
        if (state != State.FULL) switchModeToFull();
        break;

      case SHOOTING:
      case UNINITIALIZED:
        break;
    }

    if (state == State.SHOOTING) emptyMag();
  }

  /** Updates everything to do with the intake */
  private void updateIntake() {
    if (clearingIntake) { // if clearing the intake
      if (intake.timer.isFinished()) { // if done clearing the intake
        clearingIntake = false;

      } else return;

    } else if (intake.isStalled() && intake.isIntaking()) { // if intake is intaking and stalled
      clearIntake(); // eject the intake to clear the blockage
      return;
    }

    switch (state) {
      case UNINITIALISED:
        break;

      case FULL:
        if (intake.isIntaking()) {
          if (spindex.isAtTarget() && fullWait.isFinished()) clearIntake();

        } else {
          intake.ejectIdle();
        }
        break;

      case SHOOTING:
        intake.intakeIdle();
        break;

      case INTAKING:
      default:
        intake.intake();
        break;
    }
  }

  /** Updates the indicator lights */
  private void updateIndicators() {
    switch (state) {
      case UNINITIALISED:
        setIndicatorColor(RGBIndicator.Color.VIOLET);
        break;

      case SHOOTING:
        setIndicatorColor(RGBIndicator.Color.BLUE);
        break;

      case FULL:
        if (isReadyToShoot()) {
          setIndicatorColor(RGBIndicator.Color.GREEN);

        } else {
          setIndicatorColor(RGBIndicator.Color.WHITE);
        }
        break;

      case INTAKING:
        int numBalls = 0;
        RGBIndicator.Color color = RGBIndicator.Color.RED;
        for (BallColor ball : spindex.getSpindexContents()) {
          if (ball.isShootable()) numBalls++;
        }
        switch (numBalls) {
          case 1:
            color = RGBIndicator.Color.ORANGE;
            break;

          case 2:
            color = RGBIndicator.Color.YELLOW;
            break;
        }

        setIndicatorColor(color);
        break;
    }
  }

  /** Updates (or adds the data of) the telemetry from the scoring systems */
  private void updateTelemetry() {
    double avLoopTime = 0;
    double minLoopTime = Double.POSITIVE_INFINITY;
    double maxLoopTime = Double.NEGATIVE_INFINITY;
    for (Pair<Double, Double> log : loopTimes) {
      avLoopTime += log.first;
      if (log.first > maxLoopTime) maxLoopTime = log.first;
      if (log.first < minLoopTime) minLoopTime = log.first;
    }
    avLoopTime /= loopTimes.size();

    telemetry.addData("State", state.toString());
    telemetry.addData("Ready to shoot", isReadyToShoot());
    telemetry.addData("Mag", Arrays.toString(spindex.getSpindexContents()));
    telemetry.addData("Sequence", ballSequence);

    SystemReport spindexReport = spindex.getStatus();
    SystemReport turretReport = turret.getStatus();

    if (spindexReport.status == SystemStatus.NOMINAL
        && turretReport.status == SystemStatus.NOMINAL) {
      telemetry.addData("System status", "🟩Normal"); // emoji might not work

    } else {
      telemetry.addData("System status", "🟥Abnormal"); // emoji might not work
      telemetry.addData("Spindex status", spindex.getStatus().message);
      telemetry.addData("Turret status", turret.getStatus().message);
    }

    telemetry.addData("Color sensor enabled", spindex.isColorSensorEnabled());
    telemetry.addData("Angle offset", turret.getHorizontalAngleOffsetDegrees());
    telemetry.addData("Target distance", turret.getTargetDistance());
    telemetry.addData("Turret at target", turret.isAtTarget());
    telemetry.addData("Spindex at target", spindex.isAtTarget());
    telemetry.addData("Intake color", spindex.getIntakeColor());
    telemetry.addData("Spindex state", spindex.getState());
    telemetry.addData("Loop time", avLoopTime);
    telemetry.addData("Max loop time", maxLoopTime);
    telemetry.addData("Min loop time", minLoopTime);
  }

  /**
   * @brief switches the scoring system's mode to "full"
   */
  protected void switchModeToFull() {
    state = State.FULL;
    intake.intake();
    spindex.prepToShootSequence(ballSequence); // prep spindex to shoot current sequence
    turret.activate(); // get ready to shoot at any time
    fullWait.start();
  }

  /**
   * @brief switches the scoring system's mode to "intaking"
   */
  protected void switchModeToIntaking() {
    state = State.INTAKING;
    intake.intake(); // start the intake spinning
    turret.idle(); // flywheel doesn't need to at full speed
  }

  /**
   * @brief switches the scoring system's mode to "shooting"
   * @note does not do all that needs to be done when switching the mode to shooting
   */
  protected void switchModeToShooting() {
    state = State.SHOOTING;
    intake.intakeIdle(); // start the intake spinning
    turret.activate(); // start the flywheel spinning (just in case)
  }

  /**
   * @brief the internal logic for emptying the mag
   */
  protected void emptyMag() {
    if (spindex.isReadyToShoot() && turret.isReadyToShoot()) {
      spindex.shoot();
    }
  }

  /**
   * Starts shooting a sequence of balls out of the turret
   *
   * @return true if the mag wasn't empty, false if the mag is empty
   */
  public boolean shoot() {
    if (!spindex.isIndexValid(spindex.getShootableIndex())) return false;
    // ^ return false if there are no balls in the mag
    switchModeToShooting();
    emptyMag(); // start emptying mag
    return true;
  }

  /**
   * @brief gets the state of the scoring system
   * @return the state of the scoring system
   */
  public State getState() {
    return state;
  }

  /**
   * Updates the current position of the robot on the field
   * @param position the current position of the robot on the field
   * @note intended to be called every loop
   */
  public void setRobotPosition(Pose2D position) {
    robotPosition = position;
  }

  public void setBallSequence(BallSequence sequence) {
    ballSequence = sequence;
  }

  /**
   * Sets the intake as full manually
   * @param ball the color of ball in the intake
   * @note intended as a driver backup only
   */
  public void setIntakeFull(BallColor ball) {
    spindex.setIntakeColor(ball);
  }

  /**
   * Sets if the color sensor on the spindexer is enabled
   * @param enabled true if enabled, false if disabled
   * @note intended as a driver backup only
   */
  public void setColorSensorEnabled(boolean enabled) {
    spindex.setColorSensorEnabled(enabled);
  }

  /**
   * Gets if the color sensor on the spindexer is enabled
   * @return true if enabled, false if disabled
   */
  public boolean isColorSensorEnabled() {
    return spindex.isColorSensorEnabled();
  }

  /**
   * Switches if the color sensor on the spindexer is enabled
   * @note intended as a driver backup only
   */
  public void toggleColorSensorEnabled() {
    setColorSensorEnabled(!isColorSensorEnabled());
  }

  /**
   * Gets if the scoring system is ready to shoot
   *
   * @return true if ready to shoot, false otherwise
   */
  protected boolean isReadyToShoot() {
    return turret.isReadyToShoot() && spindex.isReadyToShoot() /*&& limelight.isTargetInFrame()*/;
  }

  /**
   * @brief gets the distance to the target (from the robot), in inches
   * @return the number of inches from the robot to the target
   */
  protected double getTargetDistance() {
    double distX = targetPosition.getX(DistanceUnit.INCH) - robotPosition.getX(DistanceUnit.INCH);
    double distY = targetPosition.getY(DistanceUnit.INCH) - robotPosition.getY(DistanceUnit.INCH);
    double dist = Math.hypot(Math.abs(distX), Math.abs(distY));
    return Double.isNaN(dist) ? 0 : dist;
  }

  /**
   * @brief gets the absolute angle from the robot to the target, not accounting for the robots
   *     rotation
   * @return the absolute angle from the robot to the target, in degrees
   */
  protected double getAbsoluteTargetAngle() {
    double distX = robotPosition.getX(DistanceUnit.INCH) - targetPosition.getX(DistanceUnit.INCH);
    double distY = robotPosition.getY(DistanceUnit.INCH) - targetPosition.getY(DistanceUnit.INCH);

    double angle = AngleUnit.DEGREES.fromRadians(Math.atan(distY / distX));

    return Double.isNaN(angle) ? 0 : angle; // just in case
  }

  /**
   * @brief gets the angle to the target relative to the robot, in degrees
   * @return the angle of the target relative to the robot
   * @note this value isn't normalized between -180 and 180 degrees
   */
  protected double getRelativeTargetAngle() {
    return getAbsoluteTargetAngle() - robotPosition.getHeading(AngleUnit.DEGREES) + 180;
  }

  /**
   * @brief sets the intake to eject at full speed (for some amount of time)
   * @note intended for external use only in emergency game situations (when something has
   *     malfunctioned); not intended for external use normally or regularly
   */
  public void clearIntake() {
    intake.eject(); // set intake to eject at full speed
    intake.timer.start(); // start intake timer
    clearingIntake = true; // we are clearing the intake
  }

  /**
   * @brief forces a re-check of the sequence to shoot
   * @note intended for use in emergency game situations (when something has malfunctioned); not
   *     intended to be used normally or regularly
   */
  public void emergencyRecheckSequence() {
    limelight.detectSequence();
  }

  /**
   * Forces a re read of the turret's absolute encoder
   *
   * @note intended only as a driver backup
   */
  public void reSyncTurretEncoder() {
    turret.syncEncoder();
  }

  /**
   * @brief adjusts the turret's horizontal angle offset by a given amount
   * @param offsetDegrees the amount, in degrees, to adjust the turret's horizontal angle offset by
   */
  public void adjustTurretAngleOffset(double offsetDegrees) {
    turret.changeHorizontalAngleOffsetDegrees(offsetDegrees);
  }

  /**
   * @brief gets the 2D position of the robot on the field according to limelight
   * @return the position of the robot, or null if either the target isn't visible or the camera
   *     isn't still
   * @note this returns null under normal operation conditions, be careful
   */
  public Pose2D getRobotPosition() {
    Pose2D limelightPosition = limelight.getPosition();
    if (limelightPosition == null || !turret.isAtTarget()) return null;
    double rotationDegrees = AngleUnit.normalizeDegrees(turret.getHorizontalAngleDegrees());
    double x = limelightPosition.getX(DistanceUnit.INCH);
    double y = limelightPosition.getY(DistanceUnit.INCH);
    double heading =
        AngleUnit.normalizeDegrees(
            limelightPosition.getHeading(AngleUnit.DEGREES) - rotationDegrees);
    return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
  }

  /**
   * @brief gets the state limelight is in
   * @return limelight's state
   */
  public Limelight.LimeLightMode getLimelightState() {
    return limelight.getMode();
  }

  /**
   * (Re) homes the spindexer
   *
   * @note only to be used as a driver backup
   */
  public void homeSpindexer() {
    spindex.reHome();
  }

  /**
   * @brief overrides aiming, switching control away from pinpoint / limelight
   * @param distance the distance from the target, in inches
   * @param angle the angle to turn the turret to
   */
  public void overrideAiming(double distance, double angle) {
    turretAimOverride = true;
    distanceOverride = distance;
    horizontalAngleOverride = angle;
  }

  /**
   * @brief gets if the aiming of the turret is override
   * @return false if the robot is auto aiming, true if the robot is using an aim override
   */
  public boolean isAimOverride() {
    return turretAimOverride;
  }

  /**
   * @brief intended for use when tuning the lookup table, providing manual flywheel control
   * @param RPM the RPM to spin the flywheel at
   * @param angle the angle to move the flap to
   */
  @TestOnly
  public void tuneAiming(double RPM, double angle) {
    tuning = true;
    rpmOverride = RPM;
    verticalAngleOverride = angle;
  }

  /**
   * @brief switches aiming control back to limelight (from manual override)
   */
  public void autoAim() {
    turretAimOverride = false;
  }

  /**
   * Sets the color of the indicator lights
   *
   * @param color the color to set them to
   */
  public void setIndicatorColor(RGBIndicator.Color color) {
    indicator1.setColor(color);
    indicator2.setColor(color);
  }
}
