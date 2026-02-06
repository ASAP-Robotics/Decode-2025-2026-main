package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

@Autonomous(name = "Blue Auto")
public class BlueAuto extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
      // SimpleAuto robot = new SimpleAuto(hardwareMap, telemetry, AllianceColor.BLUE);
      AutoRobot robot = new AutoRobot(hardwareMap, telemetry, AllianceColor.BLUE);

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
