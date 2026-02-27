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

package org.firstinspires.ftc.teamcode.hardware.sensors;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.LinkedList;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

@Config
public class ColorSensorV3 implements System {
  protected static class Reading {
    public final double timestamp;
    public final double distance;

    public Reading(double timestamp, double distance) {
      this.timestamp = timestamp;
      this.distance = distance;
    }
  }

  protected SystemStatus status = SystemStatus.NOMINAL;
  protected final ColorSensor colorSensor;
  protected final DistanceSensor distanceSensor;
  public static double purple = 200.0;
  public static double purpleTolerance = 24;
  public static double green = 157.0;
  public static double greenTolerance = 20;
  public static double greenHueMin = 0.6;
  protected BallColor color = BallColor.INVALID;
  protected ElapsedTime timeSinceStart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  protected LinkedList<Reading> readings = new LinkedList<>();
  protected static final double DISCONNECT_TIME = 1.0; // seconds
  public static double BALL_DISTANCE_THRESHOLD = 2.7; // inches

  // FtcDashboard dashboard = FtcDashboard.getInstance();
  // Telemetry telemetry = dashboard.getTelemetry();
  // private final Telemetry telemetry;

  public ColorSensorV3(HardwareMap hardwareMap, String deviceName /*, Telemetry telemetry*/) {
    this.colorSensor = hardwareMap.get(ColorSensor.class, deviceName);
    this.distanceSensor = hardwareMap.get(DistanceSensor.class, deviceName);
    this.colorSensor.enableLed(true);
    // this.telemetry = telemetry;
    this.timeSinceStart.reset();
  }

  /** Updates the color sensor readings, call every loop */
  public void update() {
    double distance = distanceSensor.getDistance(DistanceUnit.INCH);
    // telemetry.addData("Dist", distance);

    if (distance <= BALL_DISTANCE_THRESHOLD) {
      float[] hsv = new float[3];
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
      float h = hsv[0];
      float s = hsv[1];
      float v = hsv[2];
      // telemetry.addData("Hue", h);
      // telemetry.addData("Sat", s);
      // telemetry.addData("Val", v);

      if (Math.abs(h - green) <= greenTolerance && s >= greenHueMin) { // green
        color = BallColor.GREEN; // intake has a green ball in it

      } else if (Math.abs(h - purple) <= purpleTolerance) { // purple
        color = BallColor.PURPLE; // intake has a purple ball in it

      } else { // color can't be determined
        color = BallColor.UNKNOWN; // intake has an unknown ball in it
      }

    } else {
      color = BallColor.EMPTY;
    }

    // telemetry.update();

    double now = timeSinceStart.seconds();
    readings.add(new Reading(now, distance));
    // remove old readings
    while (!readings.isEmpty() && now - readings.getFirst().timestamp > DISCONNECT_TIME) {
      readings.removeFirst();
    }
    boolean connected = false;
    Reading previousReading = readings.isEmpty() ? null : readings.getLast();
    for (Reading reading : readings) {
      if (reading.distance != previousReading.distance) {
        connected = true;
        break;
      }
      previousReading = reading;
    }
    status = connected ? SystemStatus.NOMINAL : SystemStatus.INOPERABLE;
  }

  public SystemReport getStatus() {
    String message;
    switch (status) {
      case NOMINAL:
        message = "Color sensor operational";
        break;

      case INOPERABLE:
        message = "Color sensor disconnected";
        break;

      case FALLBACK:
        message = "Color sensor using fallback";
        break;

      default:
        message = "Color sensor in unknown state";
    }
    return new SystemReport(status, message);
  }

  /**
   * Gets the detected ball color
   *
   * @return the ball color
   * @note if the sensor is disconnected, returns invalid
   */
  public BallColor getColor() {
    return getStatus().status == SystemStatus.NOMINAL ? color : BallColor.INVALID;
  }
}
