package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ball Sorter Mag", group = "Sensor")
public class BallSorterMag extends LinearOpMode {

  private ColorSensor colorSensor;
  private Servo magServo;

  private String[] slots = new String[3];
  private int currentSlot = 0;

  private final double[] positions = {0.0, 0.33, 0.66}; // adjust for servo

  private ElapsedTime runtime = new ElapsedTime();
  private double lastMoveTime = 0;
  private final double MOVE_DELAY = 400; // ms

  private boolean waitingForClear = false; // prevents double-counting

  @Override
  public void runOpMode() {
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    magServo = hardwareMap.get(Servo.class, "magServo");

    magServo.setPosition(positions[currentSlot]);

    waitForStart();
    runtime.reset();

    while (opModeIsActive()) {
      if (runtime.milliseconds() - lastMoveTime > MOVE_DELAY) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv);

        float h = hsv[0]; // Hue
        float s = hsv[1]; // Saturation
        float v = hsv[2]; // Value

        String detected = null;

        // Detect grey/neutral → reset
        if (s < 0.2 && v < 35) {
          waitingForClear = false;
        }

        // Only detect colors if we're not waiting for grey
        if (!waitingForClear) {
          // Green detection (your measured average ~160°)
          if (s > 0.6 && v > 40 && h >= 150 && h <= 170) {
            detected = "Green";
          }
          // Purple detection (your measured average ~230°)
          else if (s > 0.3 && v > 40 && h >= 220 && h <= 240) {
            detected = "Purple";
          }

          if (detected != null) {
            slots[currentSlot] = detected;
            currentSlot = (currentSlot + 1) % 3;
            magServo.setPosition(positions[currentSlot]);
            lastMoveTime = runtime.milliseconds();

            // Now require grey before next ball
            waitingForClear = true;
          }
        }

        telemetry.addData("HSV", "H: %.1f  S: %.2f  V: %.2f", h, s, v);
        telemetry.addData("Detected", detected);
        telemetry.addData("Waiting for Grey?", waitingForClear);
      }

      telemetry.addData("Slot 0", slots[0]);
      telemetry.addData("Slot 1", slots[1]);
      telemetry.addData("Slot 2", slots[2]);
      telemetry.addData("Current Slot", currentSlot);
      telemetry.update();
    }
  }
}
