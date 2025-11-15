/*
Copyright 2024 FIRST Tech Challenge Team 22029

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

// import org.firstinspires.ftc.teamcode.ColorSense;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names of
 * OpModes appear on the menu of the FTC Driver Station. When an selection is made from the menu,
 * the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * <p>Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being added
 * to the Driver Station.
 */
@TeleOp(name = "shooter")
public class shooter extends OpMode {
  /* Declare OpMode members. */

  private DcMotor ShooterR;
  private DcMotor leftBack;
  private DcMotor rightBack;
  private DcMotor rightFront;
  private DcMotor leftFront;
  private DcMotor intake;

  // ColorSense senseColor;

  @Override
  public void init() {

    // wheels

    // ShooterR= hardwareMap.get(DcMotor.class, "ShooterR");
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");

    leftBack.setDirection(DcMotor.Direction.REVERSE);
    leftFront.setDirection(DcMotor.Direction.REVERSE);

    intake = hardwareMap.get(DcMotor.class, "intake");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {}

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {}

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {

    intake.setPower(-1);
    float y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
    double x = gamepad1.right_stick_x * 1.1; // Factor to counteract imperfect strafing
    float rx = gamepad1.left_stick_x;
    // double denominator = Math.max(Math.max(Math.abs(y), Math.abs(x)), Math.abs(rx)); // Ensure
    // max motor power ratio
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    double denominator =
        JavaUtil.maxOfList(
            JavaUtil.createListWith(
                JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))),
                1));
    // Apply control to motors
    leftFront.setPower(((y + x + rx) / denominator));
    leftBack.setPower((((y - x) + rx) / denominator));
    rightFront.setPower((((y - x) - rx) / denominator));
    rightBack.setPower((((y + x) - rx) / denominator));
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {}
}
