package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
      //SimpleAuto robot = new SimpleAuto(hardwareMap, telemetry, AllianceColor.RED);
      AutoRobot robot = new AutoRobot(hardwareMap, telemetry, AllianceColor.RED);

      robot.init();

      while (opModeInInit()) {
        robot.initLoop();
      }

      waitForStart();

      robot.start();

      while (opModeIsActive()) {
        robot.loop();
      }
    }
  }
}
