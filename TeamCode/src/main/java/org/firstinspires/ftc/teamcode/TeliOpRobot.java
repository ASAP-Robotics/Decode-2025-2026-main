/*
 * Copyright 2025 ASAP Robotics (FTC Team 22029)
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

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

/**
 * @brief class to contain the behavior of the robot in TeliOp, to avoid code duplication
 */
public class TeliOpRobot extends CommonRobot {
  protected Gamepad gamepad1;
  protected Gamepad gamepad2;
  public MecanumWheelBase wheelBase;

  public TeliOpRobot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      AllianceColor allianceColor,
      Gamepad gamepad1,
      Gamepad gamepad2) {
    super(hardwareMap, telemetry, allianceColor);

    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;

    DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
    DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
    DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
    DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
    wheelBase = new MecanumWheelBase(frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * @brief to be called once, when the opMode is initialized
   */
  public void init() {
    mag.init(false, false); // initialize scoring systems
  }

  /**
   * @brief to be called repeatedly, while the opMode is in init
   */
  public void initLoop() {
    mag.initLoop();
  }

  /**
   * @brief to be called once when the opMode is started
   */
  public void start() {
    mag.start(false); // start scoring systems up
  }

  /**
   * @brief to be called repeatedly, every loop
   */
  public void loop() {
    // get robot position
    pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
    Pose2D location = pinpoint.getPosition();

    // update scoring systems
    mag.setRobotRotation(0 /*location.getHeading(AngleUnit.DEGREES)*/);
    mag.update();

    if (gamepad2.dpadDownWasPressed()) {
      mag.setBallSequence(BallSequence.PGP);

    } else if (gamepad2.dpadLeftWasPressed()) {
      mag.setBallSequence(BallSequence.GPP);

    } else if (gamepad2.dpadRightWasPressed()) {
      mag.setBallSequence(BallSequence.PPG);
    }

    // shoot
    if (gamepad2.right_trigger > 0.5) {
      // mag.shootMag(); // shoot all balls in the mag, in a sequence if possible
      mag.shootUnsorted();

    } else if (gamepad2.rightBumperWasPressed()) {
      mag.shootSequence();
    }

    // intake
    if (gamepad2.left_trigger > 0.5) {
      mag.fillMagUnsorted(); // fill the mag with any three balls

    } else if (gamepad2.leftBumperWasPressed()) {
      mag.emergencyEject();

    } else if (gamepad2.yWasPressed()) {
      mag.intakeBall(); // intake one ball into mag
    }

    // update wheelbase
    wheelBase.setRotation(location.getHeading(AngleUnit.DEGREES)); // for field-centric control
    wheelBase.setThrottle(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
    wheelBase.update();

    // update telemetry
    telemetry.update();
  }

  /**
   * @brief to be called once, when the "stop" button is pressed
   */
  public void stop() {
    mag.stop(); // stop all powered movement in scoring systems
    wheelBase.stop(); // stop all powered movement in wheels
  }
}
