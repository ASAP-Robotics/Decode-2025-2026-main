package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {

    // Get the color sensor from the configuration
    ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

    // Array to store HSV values
    float[] hsvValues = new float[3];

    waitForStart();

    while (opModeIsActive()) {

      // Convert RGB to HSV
      Color.RGBToHSV(
          colorSensor.red() * 8, // Scale to improve accuracy
          colorSensor.green() * 8,
          colorSensor.blue() * 8,
          hsvValues);

      // Telemetry Output

      telemetry.addData("Hue", hsvValues[0]);
      telemetry.addData("Saturation", hsvValues[1]);
      telemetry.addData("Value", hsvValues[2]);
      telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.INCH));
      telemetry.update();
    }
  }
}
