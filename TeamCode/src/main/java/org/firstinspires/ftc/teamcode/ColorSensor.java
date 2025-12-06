package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ColorSensorV3;

@TeleOp
public class ColorSensor extends LinearOpMode {
  protected ColorSensorV3 colorSensorV3;

  public void runOpMode() {
    colorSensorV3 = new ColorSensorV3(hardwareMap, "colorSensor");

    waitForStart();

    colorSensorV3.start();

    while (opModeIsActive()) {
      colorSensorV3.update(telemetry);

      telemetry.addData("Color", colorSensorV3.getColor());
      telemetry.update();
    }
  }
}
