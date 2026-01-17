package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {
  private DcMotor motor1;
  private DcMotor motor2;

  @Override
  public void runOpMode() {
    motor1 = hardwareMap.get(DcMotor.class, "motor1");
    motor2 = hardwareMap.get(DcMotor.class, "motor2");
    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();

    while (opModeIsActive()) {
      motor1.setPower(gamepad1.right_stick_y);
      motor2.setPower(gamepad1.left_stick_y);
    }
  }
}
