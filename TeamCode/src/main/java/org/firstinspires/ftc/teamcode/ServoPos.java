package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;

@TeleOp
@Config
public class ServoPos extends LinearOpMode {
  public static double position = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    Axon servo = new Axon(hardwareMap.get(Servo.class, "intakeBlocker"));
    waitForStart();

    while (opModeIsActive()) {
      servo.setPosition(position);
    }
  }
}
