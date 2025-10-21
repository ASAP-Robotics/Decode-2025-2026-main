package org.firstinspires.ftc.teamcode.utils;

public class MathUtils {
  /**
   * @brief maps a number from one range to another
   * @param x the number to map
   * @param inMin the minimum of the input range
   * @param inMax the maximum of the input range
   * @param outMin the minimum of the output range
   * @param outMax the maximum of the output range
   * @return the mapped number
   */
  public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }
}
