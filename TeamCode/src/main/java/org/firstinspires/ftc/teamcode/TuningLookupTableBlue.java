package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

@TeleOp
public class TuningLookupTableBlue extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    TuningLookupTableRobot robot =
        new TuningLookupTableRobot(hardwareMap, telemetry, AllianceColor.BLUE, gamepad1, gamepad2);

    robot.init();

    while (opModeInInit()) {
      robot.initLoop();
    }

    robot.start();

    while (opModeIsActive()) {
      robot.loop();
    }

    robot.stop();
  }
}
