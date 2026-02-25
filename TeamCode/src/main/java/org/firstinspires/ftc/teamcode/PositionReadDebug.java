package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.PositionFileReader;

@TeleOp(name = "Position file", group = "test")
public class PositionReadDebug extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    waitForStart();
    PositionFileReader reader = new PositionFileReader();
    telemetry.addData("Position", reader.getPosition());
    telemetry.addData("Defaulted", reader.isDefaulted());
    telemetry.update();
    while (opModeIsActive())
      ;
  }
}
