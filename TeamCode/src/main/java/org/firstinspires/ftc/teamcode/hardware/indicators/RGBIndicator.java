/*
 * Copyright 2026 ASAP Robotics (FTC Team 22029)
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

package org.firstinspires.ftc.teamcode.hardware.indicators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utils.Follower;

/** Simple class to control a GoBILDA "RGB Indicator light (PWM controlled)" */
public class RGBIndicator {
  public enum Color {
    OFF(0.0),
    RED(0.288),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.500),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1.0);

    public final double num;

    Color(double num) {
      this.num = num;
    }
  }

  private static final double UPDATE_TOLERANCE = 0.01;
  private final Servo led; // light is controlled by servo PWM control
  private final Follower follower = new Follower(0, 0, 0, 0.15);
  private Color color = null;
  private double lastSetValue = Double.NEGATIVE_INFINITY;
  private boolean atColor = false;

  public RGBIndicator(HardwareMap hardwareMap, String deviceName) {
    this.led = hardwareMap.get(Servo.class, deviceName);
  }

  public void update() {
    if (atColor) return;
    double value = /*follower.getValue()*/ follower.getTarget();

    if (Math.abs(value - lastSetValue) > UPDATE_TOLERANCE) {
      led.setPosition(value);
      lastSetValue = value;
    }

    if (follower.isAtTarget()) atColor = true;
  }

  public RGBIndicator(Servo led) {
    this.led = led;
  }

  /**
   * Sets the color of the light
   *
   * @param color the color to set the light to
   */
  public void setColor(Color color) {
    if (color == this.color) return;
    this.color = color;
    atColor = false;
    follower.setTarget(color.num);
  }

  /**
   * Gets the currently displayed color
   *
   * @return the color being displayed
   */
  public Color getColor() {
    return color;
  }
}
