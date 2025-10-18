package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

@TeleOp(name = "Pinpoint Behavior Test", group = "Testing")
public class PinpointBehaviorTest extends LinearOpMode {

  @Override
  public void runOpMode() {
    GoBildaPinpointDriver pinpoint =
        hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

    pinpoint.update();
    telemetry.addData("Initialized", false);
    telemetry.addData("position", pinpoint.getPosition().toString());
    telemetry.addData("Frequency", pinpoint.getFrequency());
    telemetry.addData("Device Status", pinpoint.getDeviceStatus());
    telemetry.addData("Device Version", pinpoint.getDeviceVersion());
    telemetry.addData("Encoder X", pinpoint.getEncoderX());
    telemetry.addData("Encoder Y", pinpoint.getEncoderY());
    telemetry.update();

    waitForStart();

    // initialize pinpoint
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.update();

    telemetry.addData("Initialized", true);
    telemetry.addData("position", pinpoint.getPosition().toString());
    telemetry.addData("Frequency", pinpoint.getFrequency());
    telemetry.addData("Device Status", pinpoint.getDeviceStatus().toString());
    telemetry.addData("Device Version", pinpoint.getDeviceVersion());
    telemetry.addData("Encoder X", pinpoint.getEncoderX());
    telemetry.addData("Encoder Y", pinpoint.getEncoderY());
    telemetry.update();

    while (opModeIsActive()); // wait for program end
  }
}
