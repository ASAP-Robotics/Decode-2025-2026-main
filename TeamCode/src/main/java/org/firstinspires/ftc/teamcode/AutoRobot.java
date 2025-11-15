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

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

public class AutoRobot extends CommonRobot {
  // stuff (variables, etc., see TeliOpRobot) goes here; TODO: update

  public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    super(hardwareMap, telemetry, allianceColor, true, true);

    // other "Init" setup stuff goes here

  }


  public void start() {

    mag.start(true); // start scoring systems up

  }


  public void loop() {
    // other stuff goes here; TODO: fill out

    // update scoring systems
    mag.setRobotRotation(0);
    mag.update();

    // update telemetry
    telemetry.update();

    mag.shootMag();
  }


  public void stop() {

    mag.stop(); // stop all powered movement in scoring systems
  }
}
