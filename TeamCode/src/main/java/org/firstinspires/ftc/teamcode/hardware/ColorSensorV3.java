/*
 * Copyright 2025 ASAP Robotics (FTC Team 22029)
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

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.types.BallColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

public class ColorSensorV3 {
  protected final ColorSensor colorSensor;
  protected final DistanceSensor distanceSensor;
  protected BallColor color = BallColor.INVALID;
  protected ElapsedTime timeSinceBallDetected = new ElapsedTime();
  protected SimpleTimer colorReadTimer = new SimpleTimer(0.1);
  protected static final double BALL_DETECTION_TIME = 0.5; // seconds
  protected static final double BALL_DISTANCE_THRESHOLD = 1.5; // inches

  public ColorSensorV3(HardwareMap hardwareMap, String deviceName) {
    this.colorSensor = hardwareMap.get(ColorSensor.class, deviceName);
    this.distanceSensor = hardwareMap.get(DistanceSensor.class, deviceName);
    this.colorSensor.enableLed(true);
  }

  /**
   * @brief to be called one, on program start
   */
  public void start() {
    timeSinceBallDetected.reset();
    colorReadTimer.start();
  }

  /**
   * @brief updates the color sensor readings, call every loop
   */
  public void update() {
    double distance = distanceSensor.getDistance(DistanceUnit.INCH);
    if (distance <= BALL_DISTANCE_THRESHOLD) {
      if (timeSinceBallDetected.seconds() > BALL_DETECTION_TIME) {
        color = BallColor.UNKNOWN;
      }
      timeSinceBallDetected.reset();
    }

    if (timeSinceBallDetected.seconds() <= BALL_DETECTION_TIME) {
      if (colorReadTimer.isFinished()) {
        readColor();
        colorReadTimer.start();
      }

    } else {
      color = BallColor.EMPTY;
    }
  }

  /**
   * @brief gets the detected ball color
   * @return the ball color
   */
  public BallColor getColor() {
    return color;
  }

  /**
   * @brief reads the color from the color sensor and updates the ball color
   * @note this is relatively slow, don't call every loop
   */
  protected void readColor() {
    float[] hsv = new float[3];
    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
    float h = hsv[0];
    if (h >= 145 && h <= 170) { // green
      color = BallColor.GREEN; // intake has a green ball in it

    } else if (h >= 185 && h <= 205) { // purple
      color = BallColor.PURPLE; // intake has a purple ball in it

    } else { // color can't be determined
      color = BallColor.UNKNOWN; // intake has an unknown ball in it
    }
  }
}
