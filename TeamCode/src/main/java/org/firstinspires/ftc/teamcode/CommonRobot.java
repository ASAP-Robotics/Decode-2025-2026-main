/*
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029)
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

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;
import org.firstinspires.ftc.teamcode.hardware.sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.hardware.sensors.ElcAbsEncoderAnalog;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

/**
 * @brief class to contain the configuration of the robot, to avoid code duplication
 */
public abstract class CommonRobot {
  protected boolean bulkRead;
  protected List<LynxModule> allHubs; // for loop time optimization
  protected HardwareMap hardwareMap;
  protected Telemetry telemetry;
  protected AllianceColor allianceColor;
  public ScoringSystem scoringSystem;

  public CommonRobot(
      HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor, boolean bulkRead) {
    this.hardwareMap = hardwareMap;
    this.bulkRead = bulkRead;

    if (this.bulkRead) {
      allHubs = this.hardwareMap.getAll(LynxModule.class);
      for (LynxModule hub : allHubs) {
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }
    }

    this.telemetry = telemetry;
    this.allianceColor = allianceColor;

    TouchSensor spindexHomer = this.hardwareMap.get(TouchSensor.class, "spindexHomer");
    MotorEx spindexMotor = new MotorEx(hardwareMap, "spindex", Motor.GoBILDA.RPM_117);
    Axon intakeBlocker = new Axon(this.hardwareMap, "intakeBlocker", "intakeBlockerEncoder");
    ColorSensorV3 colorSensor = new ColorSensorV3(this.hardwareMap, "colorSensor");
    Spindex spindex = new Spindex(spindexMotor, spindexHomer, intakeBlocker, colorSensor);

    Servo rawTurretHood = this.hardwareMap.get(Servo.class, "turretHood");
    Axon turretHood = new Axon(rawTurretHood);
    Motor turretRotator = new Motor(hardwareMap, "turretRotator", Motor.GoBILDA.RPM_1150);
    ElcAbsEncoderAnalog turretEncoder = new ElcAbsEncoderAnalog(hardwareMap, "turretEncoder");
    DcMotorEx flywheelMotor = this.hardwareMap.get(DcMotorEx.class, "flywheel");
    Turret turret = new Turret(flywheelMotor, turretRotator, turretEncoder, turretHood, 1500);

    DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");
    ActiveIntake intake = new ActiveIntake(intakeMotor);

    RGBIndicator indicator1 = new RGBIndicator(hardwareMap, "indicator1");
    RGBIndicator indicator2 = new RGBIndicator(hardwareMap, "indicator2");

    scoringSystem =
        new ScoringSystem(
            intake, turret, spindex, indicator1, indicator2, this.allianceColor, this.telemetry);
  }

  /**
   * Clears the cache of sensor data
   *
   * @note MUST be called to get new sensor data if bulk reading enabled
   * @note returns without doing anything if bulk reading not enabled in constructor
   */
  public void clearSensorCache() {
    if (!bulkRead) return;
    // clears the cache on each hub
    for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
    }
  }

  /** To be called once, when the opMode is initialized */
  public abstract void init();

  /** To be called repeatedly, while the opMode is in init */
  public abstract void initLoop();

  /** To be called once, when the opMode is started */
  public abstract void start();

  /** To be called repeatedly, while the opMode is running */
  public abstract void loop();

  /** To be called once, when the opMode is stopped */
  public abstract void stop();
}
