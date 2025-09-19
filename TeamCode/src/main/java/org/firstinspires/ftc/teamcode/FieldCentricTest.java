package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric TeleOp", group = "Drive")
public class FieldCentricTest extends LinearOpMode {

  private DcMotor frontLeft, frontRight, backLeft, backRight;
  private IMU imu;

  @Override
  public void runOpMode() {
    // Initialize motors
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    backRight = hardwareMap.get(DcMotor.class, "backRight");

    // Reverse one side so forward is forward
    frontRight.setDirection(DcMotor.Direction.REVERSE);
    backRight.setDirection(DcMotor.Direction.REVERSE);

    // Initialize IMU
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters imuParams =
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    imu.initialize(imuParams);

    // Reset heading to 0 at init
    imu.resetYaw();

    waitForStart();

    while (opModeIsActive()) {
      // Gamepad stick inputs
      double y = -gamepad1.left_stick_y; // Forward/back
      double x = gamepad1.left_stick_x; // Strafe
      double rx = gamepad1.right_stick_x; // Rotation

      // Read robot heading in radians
      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      // Rotate joystick input by -heading (field-centric transform)
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

      // Calculate motor powers
      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
      double flPower = (rotY + rotX + rx) / denominator;
      double blPower = (rotY - rotX + rx) / denominator;
      double frPower = (rotY - rotX - rx) / denominator;
      double brPower = (rotY + rotX - rx) / denominator;

      // Apply powers
      frontLeft.setPower(flPower);
      backLeft.setPower(blPower);
      frontRight.setPower(frPower);
      backRight.setPower(brPower);

      // Telemetry
      telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
      telemetry.update();
    }
  }
}
