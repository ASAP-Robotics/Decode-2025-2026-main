package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.sensors.ElcAbsEncoder;

@TeleOp
public class EncoderTest extends LinearOpMode {
  @Override
  public void runOpMode() {
    ElcAbsEncoder encoder = new ElcAbsEncoder(hardwareMap, "turretEncoder", "turretRotator");

    waitForStart();

    while (opModeIsActive()) {
      double angle = encoder.getPosition();

      if (gamepad1.aWasPressed()) {
        encoder.synchronize();
      }

      telemetry.addData("Angle (degrees)", angle);
      telemetry.update();
    }
  }
}
