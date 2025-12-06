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

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.ColorSensorV3;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

/**
 * @brief class to contain the configuration of the robot, to avoid code duplication
 */
public abstract class CommonRobot {
  protected HardwareMap hardwareMap;
  protected Telemetry telemetry;
  protected AllianceColor allianceColor;
  public ScoringSystem scoringSystem;

  public CommonRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;
    this.allianceColor = allianceColor;

    Limelight3A rawLimelight = this.hardwareMap.get(Limelight3A.class, "limelight");
    Limelight limelight = new Limelight(rawLimelight, this.allianceColor, 0.001);

    AnalogInput lifterEncoder = this.hardwareMap.get(AnalogInput.class, "lifterEncoder");
    Servo rawLifterServo2 = this.hardwareMap.get(Servo.class, "lifter2");
    Axon lifterServo2 = new Axon(rawLifterServo2, lifterEncoder);
    Servo rawLifterServo1 = this.hardwareMap.get(Servo.class, "lifter1");
    rawLifterServo1.setDirection(Servo.Direction.REVERSE);
    Axon lifterServo1 = new Axon(rawLifterServo1);
    AnalogInput magServoEncoder = this.hardwareMap.get(AnalogInput.class, "magServoEncoder");
    Servo rawMagServo2 = this.hardwareMap.get(Servo.class, "magServo2");
    Axon magServo2 = new Axon(rawMagServo2, magServoEncoder);
    Servo rawMagServo1 = this.hardwareMap.get(Servo.class, "magServo1");
    Axon magServo1 = new Axon(rawMagServo1, magServoEncoder);
    ColorSensorV3 colorSensor = new ColorSensorV3(this.hardwareMap, "colorSensor");
    Spindex spindex = new Spindex(magServo1, magServo2, lifterServo1, lifterServo2, colorSensor);

    Servo rawTurretHood = this.hardwareMap.get(Servo.class, "turretHood");
    Axon turretHood = new Axon(rawTurretHood);
    Motor turretRotator = new Motor(hardwareMap, "turretRotator", Motor.GoBILDA.RPM_1150);
    DcMotorEx flywheelMotor = this.hardwareMap.get(DcMotorEx.class, "flywheel");
    Turret turret = new Turret(flywheelMotor, turretRotator, turretHood, 1500);

    DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");
    ActiveIntake intake = new ActiveIntake(intakeMotor);

    scoringSystem =
        new ScoringSystem(intake, turret, spindex, limelight, this.allianceColor, this.telemetry);
  }

  /**
   * @brief to be called once, when the opMode is initialized
   */
  public abstract void init();

  /**
   * @brief to be called repeatedly, while the opMode is in init
   */
  public abstract void initLoop();

  /**
   * @brief to be called once, when the opMode is started
   */
  public abstract void start();

  /**
   * @brief to be called repeatedly, while the opMode is running
   */
  public abstract void loop();

  /**
   * @brief to be called once, when the opMode is stopped
   */
  public abstract void stop();
}
