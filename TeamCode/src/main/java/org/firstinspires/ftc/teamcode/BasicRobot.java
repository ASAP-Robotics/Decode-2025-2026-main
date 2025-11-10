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

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

/**
 * @brief class to contain the behavior of the robot, to avoid code duplication
 */
public class BasicRobot {
  protected HardwareMap hardwareMap;
  protected Telemetry telemetry;
  protected GoBildaPinpointDriver pinpoint;
  public ScoringSystem mag;

  public BasicRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor, boolean preloaded) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;
    pinpoint = this.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

    Limelight3A rawLimelight = this.hardwareMap.get(Limelight3A.class, "limelight");
    Limelight limelight = new Limelight(rawLimelight, allianceColor, false);

    AnalogInput lifterServoEncoder = this.hardwareMap.get(AnalogInput.class, "rampServoEncoder");
    Servo rawLifterServo2 = this.hardwareMap.get(Servo.class, "lifter2");
    Axon lifterServo2 = new Axon(rawLifterServo2, lifterServoEncoder);
    Servo rawLifterServo1 = this.hardwareMap.get(Servo.class, "lifter1");
    Axon lifterServo1 = new Axon(rawLifterServo1, lifterServoEncoder);
    AnalogInput magServoEncoder = this.hardwareMap.get(AnalogInput.class, "magServoEncoder");
    Servo rawMagServo2 = this.hardwareMap.get(Servo.class, "magServo2");
    Axon magServo2 = new Axon(rawMagServo2, magServoEncoder);
    Servo rawMagServo1 = this.hardwareMap.get(Servo.class, "magServo1");
    Axon magServo1 = new Axon(rawMagServo1, magServoEncoder);
    ColorSensor colorSensor = this.hardwareMap.get(ColorSensor.class, "colorSensor");
    DistanceSensor distanceSensor = this.hardwareMap.get(DistanceSensor.class, "colorSensor");
    Spindex spindex = new Spindex(magServo1, magServo2, lifterServo1, lifterServo2, colorSensor, distanceSensor);

    AnalogInput turretHoodEncoder = this.hardwareMap.get(AnalogInput.class, "turretHoodEncoder");
    Servo rawTurretHood = this.hardwareMap.get(Servo.class, "turretHood");
    Axon turretHood = new Axon(rawTurretHood, turretHoodEncoder);
    DcMotorEx turretRotator = this.hardwareMap.get(DcMotorEx.class, "turretRotator");
    DcMotorEx flywheelMotor = this.hardwareMap.get(DcMotorEx.class, "flywheel");
    Turret turret = new Turret(flywheelMotor, turretRotator, turretHood);

    DcMotorEx intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake");
    ActiveIntake intake = new ActiveIntake(intakeMotor);

    mag = new ScoringSystem(intake, turret, spindex, limelight, allianceColor, this.telemetry);
    mag.init(preloaded); // initialize scoring systems
  }
}
