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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONException;
import org.json.JSONObject;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;

@TeleOp(name = "File usage test", group = "Testing")
public class FileUsageTest extends LinearOpMode {

  @Override
  public void runOpMode() throws RuntimeException {
    File file = AppUtil.getInstance().getSettingsFile("test.json");

    JSONObject obj = new JSONObject();

    try {
      obj.put("test_1", 12.3);
    } catch (JSONException e) {
      throw new RuntimeException(e);
    }

    try {
      obj.put("test_2", true);
    } catch (JSONException e) {
      throw new RuntimeException(e);
    }

    ReadWriteFile.writeFile(file, obj.toString());

    telemetry.addData("File written", file.toString());
    telemetry.addData("Waiting", "true");
    telemetry.update();
    sleep(5000); // wait

    // Later
    String data = ReadWriteFile.readFile(file);
    JSONObject saved;

    try {
      saved = new JSONObject(data);
    } catch (JSONException e) {
      throw new RuntimeException(e);
    }

    double test1;
    boolean test2;

    try {
      test1 = saved.getDouble("test_1");
      test2 = saved.getBoolean("test_2");
    } catch (JSONException e) {
      throw new RuntimeException(e);
    }

    telemetry.addData("test_1", test1);
    telemetry.addData("test_2", test2);
    telemetry.update();

    while (opModeIsActive()); // wait for program to stop
  }
}
