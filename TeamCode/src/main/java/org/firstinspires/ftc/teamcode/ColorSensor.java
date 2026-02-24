package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;
import org.firstinspires.ftc.teamcode.hardware.sensors.ColorSensorV3;

@TeleOp
@Config
public class ColorSensor extends LinearOpMode {
  public static RGBIndicator.Color lightColor = RGBIndicator.Color.GREEN;
  protected ColorSensorV3 colorSensorV3;
  protected RGBIndicator light1;
  protected RGBIndicator light2;

  public void runOpMode() {
    colorSensorV3 = new ColorSensorV3(hardwareMap, "colorSensor");
    light1 = new RGBIndicator(hardwareMap, "indicator1");
    light2 = new RGBIndicator(hardwareMap, "indicator2");

    waitForStart();

    while (opModeIsActive()) {
      light1.setColor(lightColor);
      light2.setColor(lightColor);
      light1.update();
      light2.update();
      colorSensorV3.update();

      telemetry.addData("Color", colorSensorV3.getColor());
      telemetry.addData("Status", colorSensorV3.getStatus().message);
      telemetry.update();
    }
  }
}
