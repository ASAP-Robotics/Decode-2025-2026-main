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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;
import java.util.LinkedList;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.BallSequenceFileReader;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;
import org.jetbrains.annotations.TestOnly;
@Config
public class ScoringSystem {
  public enum State {
    UNINITIALISED,
    FULL,
    INTAKING,
    SHOOTING
  }
  public static double ballTime = 1;

  private final double ballTimeSlope =0.00715145; // the slope for the ball time equation
  private final double ballTimeOffset = -0.0353098; // the offset for the ball time equation
  public static double balltimeConstant = 0.05;
  private final ActiveIntake intake; // the intake on the robot
  private final Turret turret; // the flywheel on the robot
  protected Turret.LookupTableItem[] LOOKUP_TABLE;
  private final Spindex spindex; // the spindex on the robot
  private final RGBIndicator indicator1; // first indicator light
  private final RGBIndicator indicator2; // second indicator light
  protected State state = State.UNINITIALISED; // the state of the scoring system
  private BallSequence ballSequence = BallSequence.GPP; // the sequence being shot
  public final AllianceColor allianceColor; // the alliance we are on
  // private boolean turretAimOverride = false; // if the aim of the turret is overridden
  private boolean turretVerticalAngleOverride = false;
  private boolean turretHorizontalAngleOverride = false;
  private boolean turretRPMOverride = false;
  private boolean turretDistanceOverride = false;
  private double horizontalAngleOverride = 0;
  // private boolean tuning = false;
  private boolean shutDown = false; // if the systems are shutting down at the end of auto
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
      RGBIndicator indicator1,
      RGBIndicator indicator2,
      AllianceColor allianceColor,
      Telemetry telemetry) {
    this.intake = intake;
    this.turret = turret;
    this.spindex = spindex;
    this.indicator1 = indicator1;
    this.indicator2 = indicator2;
    this.allianceColor = allianceColor;
    this.telemetry = telemetry;
    this.targetPosition = this.allianceColor.getTargetLocation();
    this.ballSequence = new BallSequenceFileReader().getSequence();
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
    if (auto) {
      turret.init(allianceColor.getObeliskOffset());
    } else {
      turret.init(0);
    }
    turret.setActive(!isPreloaded);
    LOOKUP_TABLE = turret.fillLookupTable();
  }

  /** To be called repeatedly while the robot is in init */
  public void initLoop() {
    spindex.update();
    turret.update();
    updateIndicators();
  }

  /**
   * Starts scoring systems up
   *
   * @note call when OpMode is started ("Start" is pressed)
   */
  public void start(boolean isPreloaded) {
    spindex.start();
    turret.enable(); // let the flywheel spin up
    turret.start();

    if (isPreloaded) {
      switchModeToFull();
      intake.stop();
      clearingIntake = true;
      intake.timer.start();

    } else {
      switchModeToIntaking();
    }

    timeSinceStart.reset();
    loopTime.reset();
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
    updateAiming();
    updateSpindex();
    updateIntake();
    intake.update();
    turret.update();
    spindex.update();
    updateIndicators();
    if (updateTelemetry) updateTelemetry();
    double now = timeSinceStart.milliseconds();
    loopTimes.addLast(new Pair<>(loopTime.milliseconds(), now));
    loopTime.reset();
    while (!loopTimes.isEmpty() && now - loopTimes.getFirst().second > 1000) {
      loopTimes.removeFirst();
    }
  }

  /** Updates everything to do with aiming the turret */
  private void updateAiming() {
    if (shutDown) {
      turret.setHorizontalAngle(0);
      turret.idle();
      return;
    }

    if (turretHorizontalAngleOverride) {
      turret.setHorizontalAngle(horizontalAngleOverride);

    } else {
      turret.setHorizontalAngle(getRelativeTargetAngle());
    }

    if (turretDistanceOverride) {
      turret.setTargetDistance(distanceOverride);

    } else {
      turret.setTargetDistance(getTargetDistance());
    }

    if (turretVerticalAngleOverride) {
      turret.overrideVerticalAngle(verticalAngleOverride);
    }

    if (turretRPMOverride) {
      turret.overrideRpm(rpmOverride);
    }
  }

  /** Updates everything to do with the spindexer */
  private void updateSpindex() {
    spindex.setSequence(ballSequence);
    if (state == State.SHOOTING && isReadyToShoot() && !shutDown) {
      spindex.shoot();
    }

    switch (spindex.getState()) {
      case INTAKING:
        if (state != State.INTAKING) switchModeToIntaking();
        break;

      case SHOOTING_READY:
        if (state != State.FULL && state != State.SHOOTING) switchModeToFull();
        break;

      case SHOOTING:
      case UNINITIALIZED:
        break;
    }
  }

  /** Updates everything to do with the intake */
  private void updateIntake() {
    if (shutDown) {
      intake.stop();
      return;
    }

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
      case SHOOTING:
        if (intake.isIntaking()) {
          if (spindex.isAtTarget() && fullWait.isFinished()) clearIntake();

        } else {
          intake.ejectIdle();
        }
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
        if (spindex.isAtTarget() && turret.isAtTarget()) {
          setIndicatorColor(RGBIndicator.Color.GREEN);
        } else {
          setIndicatorColor(RGBIndicator.Color.VIOLET);
        }
        break;

      case SHOOTING:
        setIndicatorColor(RGBIndicator.Color.WHITE);
        break;

      case FULL:
        if (isReadyToShoot()) {
          setIndicatorColor(RGBIndicator.Color.GREEN);

        } else {
          setIndicatorColor(RGBIndicator.Color.BLUE);
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

    indicator1.update();
    indicator2.update();
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
    telemetry.addData("Loop time (ms)", avLoopTime);
    telemetry.addData("Max loop time (ms)", maxLoopTime);
    telemetry.addData("Min loop time (ms)", minLoopTime);
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

  protected double getBallTime(double distance) {
    try {
      int indexOver = LOOKUP_TABLE.length - 1;
      int indexUnder = 0;
      for (int i = 0; i < LOOKUP_TABLE.length; i++) {
        if (LOOKUP_TABLE[i].getDistance() >= distance) {
          indexOver = i;
          indexUnder = indexOver - 1; // assuming values go from low to high
          break;
        }
      }

      return  balltimeConstant + MathUtils.map(
              distance,
              LOOKUP_TABLE[indexUnder].getDistance(),
              LOOKUP_TABLE[indexOver].getDistance(),
              LOOKUP_TABLE[indexUnder].getBallTime(),
              LOOKUP_TABLE[indexOver].getBallTime());
    } catch (Exception e) { // most probably if distance is outside of lookup table
      return 0;
    }

  }

  /**
   * Starts shooting a sequence of balls out of the turret
   *
   * @return true if the mag wasn't empty, false if the mag is empty
   */
  public boolean shoot() {
    if (spindex.isEmpty()) return false;
    // ^ return false if there are no balls in the mag
    spindex.prepToShootSequence(ballSequence);
    state = State.SHOOTING;
    intake.ejectIdle(); // start the intake spinning
    turret.activate(); // start the flywheel spinning
    return true;
  }

  /**
   * Cancels any shot that may be being performed
   *
   * @note takes no action whatsoever if state isn't SHOOTING
   * @note intended only as a driver backup
   */
  public void cancelShot() {
    if (state == State.SHOOTING) {
      state = State.FULL;
      spindex.cancelShot();
    }
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
   *
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
   *
   * @param ball the color of ball in the intake
   * @note intended as a driver backup only
   */
  public void setIntakeFull(BallColor ball) {
    spindex.setIntakeColor(ball);
  }

  /**
   * Manually sets the spindex as empty
   *
   * @note only to be used as a manual backup
   */
  public void setSpindexEmpty() {
    spindex.setEmpty();
  }

  /**
   * Sets if the color sensor on the spindexer is enabled
   *
   * @param enabled true if enabled, false if disabled
   * @note intended as a driver backup only
   */
  public void setColorSensorEnabled(boolean enabled) {
    spindex.setColorSensorEnabled(enabled);
  }

  /**
   * Gets if the color sensor on the spindexer is enabled
   *
   * @return true if enabled, false if disabled
   */
  public boolean isColorSensorEnabled() {
    return spindex.isColorSensorEnabled();
  }

  /**
   * Switches if the color sensor on the spindexer is enabled
   *
   * @note intended as a driver backup only
   */
  public void toggleColorSensorEnabled() {
    setColorSensorEnabled(!isColorSensorEnabled());
  }

  /**
   * Disables the spindexer motor for a bit to let any jams clear
   *
   * @note intended only as a driver backup
   */
  public void unJamSpindexer() {
    spindex.unJam();
  }

  /**
   * Gets if the scoring system is ready to shoot
   *
   * @return true if ready to shoot, false otherwise
   */
  protected boolean isReadyToShoot() {
    return turret.isReadyToShoot() && spindex.isReadyToShoot();
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
   * Gets the angle of the turret relative to straight
   *
   * @return the angle of the turret, in degrees
   */
  public double getTurretAngle() {
    return turret.getHorizontalAngleDegrees();
  }

  public Pose2D getVirtualRobotPosition(
          Pose2D robotPose,
          Pose2D targetPosition,
          double robotVelX,   // in/s (same frame as robotPosition)
          double robotVelY
  ) {
    if(Math.abs(robotVelX)<2 && Math.abs(robotVelY)<2) return robotPose;



    // distance from REAL robot to target (inches)
    double dx0 = targetPosition.getX(DistanceUnit.INCH) - robotPose.getX(DistanceUnit.INCH);
    double dy0 = targetPosition.getY(DistanceUnit.INCH) - robotPose.getY(DistanceUnit.INCH);
    double distance = Math.hypot(dx0, dy0);

    ballTime = getBallTime(distance);


    // time-of-flight estimate (seconds) — clamp so it can't go negative
    double h = robotPose.getHeading(AngleUnit.RADIANS);
    double cos = Math.cos(h);
    double sin = Math.sin(h);

// If robotVelX = forward, robotVelY = left:
    double fieldVelX = robotVelX * cos - robotVelY * sin;
    double fieldVelY = robotVelX * sin + robotVelY * cos;

    // how far robot moves during flight (inches)
    double leadX = fieldVelX * ballTime;
    double leadY = fieldVelY * ballTime;



    // VIRTUAL ROBOT = where the robot will be after ballTime
    double virtualX = robotPose.getX(DistanceUnit.INCH) + leadX;
    double virtualY = robotPose.getY(DistanceUnit.INCH) + leadY;


    // field aim angle FROM virtual robot TO real target
    double dx = targetPosition.getX(DistanceUnit.INCH) - virtualX;
    double dy = targetPosition.getY(DistanceUnit.INCH) - virtualY;


    Pose2D virtual = new Pose2D(
            DistanceUnit.INCH,
            virtualX, virtualY,
            AngleUnit.DEGREES,
            robotPose.getHeading(AngleUnit.DEGREES));


    // store aimRad in the pose heading (since that's what you want)
    return virtual;
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
   * Prepares systems for Autonomous shutdown
   *
   * @note only to be called at the end of Auto OpModes
   */
  public void prepForShutdown() {
    spindex.prepForShutdown();
    shutDown = true;
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
   * @brief overrides aiming, switching control away from pinpoint / odometry math
   * @param distance the distance from the target, in inches
   * @param angle the angle to turn the turret to
   */
  public void overrideAiming(double distance, double angle) {
    turretDistanceOverride = true;
    distanceOverride = distance;
    turretHorizontalAngleOverride = true;
    horizontalAngleOverride = angle;
  }

  /**
   * Manually sets the aiming values for Auto
   *
   * @param horizontalAngle the horizontal angle for the turret to go to
   * @param verticalAngle the angle of the hood flap
   * @param RPM the flywheel speed
   */
  public void manualAimForAuto(double horizontalAngle, double verticalAngle, double RPM) {
    turretHorizontalAngleOverride = true;
    horizontalAngleOverride = horizontalAngle;
    turretVerticalAngleOverride = true;
    verticalAngleOverride = verticalAngle;
    turretRPMOverride = true;
    rpmOverride = RPM;
  }

  /**
   * @brief gets if the aiming of the turret is overridden at all
   * @return false if the robot is auto aiming, true if the robot is using an aim override
   */
  public boolean isAimOverride() {
    return turretDistanceOverride
        || turretRPMOverride
        || turretHorizontalAngleOverride
        || turretVerticalAngleOverride;
  }

  /**
   * @brief intended for use when tuning the lookup table, providing manual flywheel control
   * @param RPM the RPM to spin the flywheel at
   * @param angle the angle to move the flap to
   */
  @TestOnly
  public void tuneAiming(double RPM, double angle) {
    turretRPMOverride = true;
    rpmOverride = RPM;
    turretVerticalAngleOverride = true;
    verticalAngleOverride = angle;
  }

  /**
   * @brief switches all aiming control back to automatic control (from manual override)
   */
  public void autoAim() {
    turretVerticalAngleOverride = false;
    turretHorizontalAngleOverride = false;
    turretRPMOverride = false;
    turretDistanceOverride = false;
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
