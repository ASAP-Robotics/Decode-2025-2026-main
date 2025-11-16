package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

@Autonomous(name = "AUTOMOTOMONOUS")
public class AUTOMOTOMONOUS extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Pose2d beginPose = new Pose2d(0, 0, 0);
    if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
      MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
      AutoRobot robot = new AutoRobot(hardwareMap, telemetry, AllianceColor.RED);

      robot.init();

      while (opModeInInit()) {
        robot.initLoop();
      }

      waitForStart();

      robot.start();

      while (opModeIsActive()) {
        robot.loop(drive);
      }
    }
  }
}
