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
  protected BallColor color = BallColor.INVALID;
  protected ElapsedTime timeSinceStart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  protected LinkedList<Reading> readings = new LinkedList<>();
  protected static final double DISCONNECT_TIME = 1.0; // seconds
  protected static final double BALL_DISTANCE_THRESHOLD = 2.5; // inches

  public ColorSensorV3(HardwareMap hardwareMap, String deviceName) {
    this.colorSensor = hardwareMap.get(ColorSensor.class, deviceName);
    this.distanceSensor = hardwareMap.get(DistanceSensor.class, deviceName);
    this.colorSensor.enableLed(true);
    this.timeSinceStart.reset();
  }

  /**
   * @brief updates the color sensor readings, call every loop
   */
  public void update() {
    double distance = distanceSensor.getDistance(DistanceUnit.INCH);
    //    telemetry.addData("Dist", distance);

    if (distance <= BALL_DISTANCE_THRESHOLD) {
      float[] hsv = new float[3];
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
      float h = hsv[0];
      //      telemetry.addData("Hue", hsv[0]);
      //      telemetry.addData("Sat", hsv[1]);
      //      telemetry.addData("Val", hsv[2]);
      if (h >= 130 && h <= 195) { // green
        color = BallColor.GREEN; // intake has a green ball in it

      } else if (h >= 195 && h <= 260) { // purple
        color = BallColor.PURPLE; // intake has a purple ball in it

      } else { // color can't be determined
        color = BallColor.UNKNOWN; // intake has an unknown ball in it
      }

    } else {
      color = BallColor.EMPTY;
    }

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
   * @brief gets the detected ball color
   * @return the ball color
   * @note if the sensor is disconnected, returns invalid
   */
  public BallColor getColor() {
    return getStatus().status == SystemStatus.NOMINAL ? color : BallColor.INVALID;
  }
}
