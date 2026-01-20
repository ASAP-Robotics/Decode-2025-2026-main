package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;

@TeleOp
public class BreakBeamSensorTest extends LinearOpMode {
  private BreakBeam sensor;

  public void runOpMode() {
    sensor = new BreakBeam(hardwareMap.get(DigitalChannel.class, "breakBeam"));

    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Beam", sensor.getState());
      telemetry.update();
    }
  }
}
