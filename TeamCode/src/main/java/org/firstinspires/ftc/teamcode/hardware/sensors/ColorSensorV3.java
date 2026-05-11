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
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

@Config
public class ColorSensorV3 implements System {
  // config vars (FTC Dashboard)
  public static boolean SHOW_TELEMETRY = false;
  public static double PURPLE_HUE = 200.0;
  public static double PURPLE_HUE_TOLERANCE = 30;
  public static double GREEN_HUE = 157.0;
  public static double GREEN_HUE_TOLERANCE = 20;
  public static double GREEN_SATURATION_MIN = 0.6;
  public static double BALL_DISTANCE_THRESHOLD = 2.3; // inches
  public static double BREAK_BEAM_TIMEOUT = 0.05; // seconds
  public static double DISCONNECT_TIME = 1.0; // seconds

  protected SystemStatus status = SystemStatus.NOMINAL;
  // maybe use NormalizedColorSensor
  protected final ColorSensor colorSensor;
  protected final DistanceSensor distanceSensor;
  protected final BreakBeam breakBeam;
  protected BallColor color = BallColor.INVALID;
  protected final ElapsedTime breakBeamTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  protected final ElapsedTime readingChangeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  protected double lastDistance = Double.POSITIVE_INFINITY;

  private final Telemetry telemetry;

  public ColorSensorV3(HardwareMap hardwareMap, String colorSensorName, String breakBeamName) {
    this.colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
    this.distanceSensor = hardwareMap.get(DistanceSensor.class, colorSensorName);
    this.breakBeam = new BreakBeam(hardwareMap.get(DigitalChannel.class, breakBeamName));
    this.colorSensor.enableLed(true);
    this.telemetry = FtcDashboard.getInstance().getTelemetry();
  }

  /** Updates the color sensor readings, call every loop */
  public void update() {
    if (breakBeam.isBroken()) breakBeamTimer.reset();

    boolean beamBroken = breakBeamTimer.seconds() <= BREAK_BEAM_TIMEOUT;
    double distance = distanceSensor.getDistance(DistanceUnit.INCH);

    if (SHOW_TELEMETRY) {
      telemetry.addData("Dist", distance);
      telemetry.addData("Beam broken", beamBroken);
    }

    if (beamBroken /*|| distance <= BALL_DISTANCE_THRESHOLD*/) {
      float[] hsv = new float[3];
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
      float h = hsv[0];
      float s = hsv[1];
      float v = hsv[2];

      if (SHOW_TELEMETRY) {
        telemetry.addData("Hue", h);
        telemetry.addData("Sat", s);
        telemetry.addData("Val", v);
      }

      if (Math.abs(h - GREEN_HUE) <= GREEN_HUE_TOLERANCE && s >= GREEN_SATURATION_MIN) { // green
        color = BallColor.GREEN; // intake has a green ball in it

      } else if (Math.abs(h - PURPLE_HUE) <= PURPLE_HUE_TOLERANCE) { // purple
        color = BallColor.PURPLE; // intake has a purple ball in it

      } else { // color can't be determined
        color = BallColor.UNKNOWN; // intake has an unknown ball in it
      }

    } else {
      color = BallColor.EMPTY;
    }

    if (SHOW_TELEMETRY) {
      telemetry.addData("Color", color);
      telemetry.update();
    }

    if (distance != lastDistance) readingChangeTimer.reset();
    lastDistance = distance;

    boolean connected = readingChangeTimer.seconds() <= DISCONNECT_TIME;

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
