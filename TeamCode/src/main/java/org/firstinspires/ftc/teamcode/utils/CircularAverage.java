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

package org.firstinspires.ftc.teamcode.utils;

import org.jetbrains.annotations.TestOnly;

/**
 * Simple class to encapsulate a fixed-size circular buffer of the `double` type, with no way to
 * read values in any specific order.
 * This is intended to be used for averaging a fixed number of values.
 */
public class CircularAverage {
  public final int length; // number of elements in buffer
  private final double[] buffer; // actual buffer
  private double sum = 0.0; // a running sum of all values in the buffer
  private int index = 0; // the next index in the buffer to write to
  private boolean full = false;

  public CircularAverage(int length) {
    this.length = length;
    this.buffer = new double[length];
  }

  /**
   * Writes a value to the buffer
   *
   * @param value the value to add. Must be finite and valid (not NaN), or will be ignored
   */
  public void write(double value) {
    if (Double.isNaN(value) || Double.isInfinite(value)) return;

    sum -= buffer[index]; // subtract old value from sum
    buffer[index] = value; // write new value to buffer
    sum += value; // add new value to sum

    if (++index >= length) { // increment index
      index = 0; // wrap the index
      full = true; // record that we have filled the buffer
    }
  }

  /**
   * Gets the average value of all elements in the buffer
   *
   * @return the average of all values in the buffer
   */
  public double average() {
    if (!full && index == 0) return 0.0; // return 0 if no elements written to buffer
    return sum / (full ? length : index);
  }

  /**
   * Gets if the buffer is full (if old values are being overwritten)
   * @return if the buffer is full
   */
  public boolean isFull() {
    return full;
  }

  /**
   * Gets a copy of the internal buffer
   * @return the internal buffer
   */
  @TestOnly
  public double[] getBuffer() {
    return buffer;
  }

  /**
   * Gets the current internal index
   * @return the current internal index
   */
  @TestOnly
  public int getIndex() {
    return index;
  }
}
