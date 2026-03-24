/*
 * Copyright 2026 ASAP Robotics (FTC Team 22029)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import java.util.List;

@TeleOp(group = "2nd robot", name = "2nd Robot Main")
@Config
public class SecondRobotMain extends LinearOpMode {
  // FtcDashboard tuning variables
  // TODO: tune
  public static double VELOCITY = 0.0;
  public static double
      KP = 0.0,
      KI = 0.0,
      KD = 0.0,
      KF = 0.0;

  private double oldVelocity = VELOCITY;
  private double
      oldKP = KP,
      oldKI = KI,
      oldKD = KD,
      oldKF = KF;

  public void runOpMode() {
    // bulk reading code
    List<LynxModule> allHubs = this.hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
    intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intake.setPower(0);

    DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setVelocity(0.0, AngleUnit.DEGREES);

    DcMotorEx // wheel motors
        fl = hardwareMap.get(DcMotorEx.class, "fl"),
        fr = hardwareMap.get(DcMotorEx.class, "fr"),
        bl = hardwareMap.get(DcMotorEx.class, "bl"),
        br = hardwareMap.get(DcMotorEx.class, "br");
    MecanumWheelBase wheelBase = new MecanumWheelBase(fl, fr, bl, br);

    waitForStart();

    while (opModeIsActive()) {
      wheelBase.setThrottle(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
      wheelBase.update(false, gamepad1.left_bumper); // update wheels

      intake.setPower(gamepad1.right_trigger); // set intake power

      if (oldKP != KP || oldKI != KI || oldKD != KD || oldKF != KF)
        flywheel.setVelocityPIDFCoefficients(KP, KI, KD, KF); // set PIDF constants if changed
      // might want to be able to turn off
      if (oldVelocity != VELOCITY)
        flywheel.setVelocity(VELOCITY, AngleUnit.DEGREES); // set velocity if changed

      // telemetry
      dashboardTelemetry.addData("Deg/s (actual)", flywheel.getVelocity(AngleUnit.DEGREES));
      dashboardTelemetry.addData("Deg/s (target)", VELOCITY);
      dashboardTelemetry.update();

      oldVelocity = VELOCITY;
      oldKP = KP;
      oldKI = KI;
      oldKD = KD;
      oldKF = KF;
    }

    wheelBase.stop();
  }
}
