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

package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.TestOnly;
import java.util.LinkedList;
import java.util.List;

/**
 * Analog-only driver for the ELC Encoder V2.
 *
 * <p>- Uses ONLY the analog absolute output Supports inversion
 */
@Config
public class ElcAbsEncoderAnalog {
  public static int SENSOR_READS = 3; // number of times to read the analog encoder
  private final AnalogInput absoluteEncoder;
  private boolean inverted = false;

  /**
   * Constructor
   *
   * @param hardwareMap OpMode hardware map
   * @param analogName Name of the Analog Input device
   */
  public ElcAbsEncoderAnalog(HardwareMap hardwareMap, String analogName) {
    this.absoluteEncoder = hardwareMap.get(AnalogInput.class, analogName);
  }

  /** Returns raw absolute angle (0–360) */
  public double getAngle() {
    List<Double> voltages = new LinkedList<>();

    for (int i = 0; i < SENSOR_READS; i++) {
      voltages.add(absoluteEncoder.getVoltage());
    }

    double voltage = 0;

    for (Double reading : voltages) {
      voltage += reading;
    }

    voltage /= voltages.size();

    voltage = Range.clip(voltage, 0.0, absoluteEncoder.getMaxVoltage());

    double angle = (voltage / absoluteEncoder.getMaxVoltage()) * 360.0;

    if (inverted) {
      angle = 360.0 - angle;
    }

    return angle;
  }

  /** Returns position normalized to [-180, 180) */
  public double getAngleNormalized() {
    return AngleUnit.normalizeDegrees(getAngle());
  }

  /** Inverts encoder direction. */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  /** Debug helper */
  @TestOnly
  public double getRawVoltage() {
    return absoluteEncoder.getVoltage();
  }
}
