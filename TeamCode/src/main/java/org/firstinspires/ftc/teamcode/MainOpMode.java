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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

@TeleOp(name = "Main TeliOp", group = "Drive")
public class MainOpMode extends LinearOpMode {
  private DcMotorEx frontLeft,
      frontRight,
      backLeft,
      backRight,
      flywheelMotor,
      intakeMotor,
      turretRotator;
  private Servo rawRampServo, turretHood;
  private CRServo rawMagServo1, rawMagServo2;
  private AnalogInput magServoEncoder, rampServoEncoder;
  private MonodirectionalDualServo magServo;
  private EncoderServo rampServo;
  private GoBildaPinpointDriver pinpoint;
  private ColorSensor colorSensor;
  private DistanceSensor distanceSensor;
  private Turret turret;
  private ActiveIntake intake;
  private Spindex spindex;
  private Camera camera;
  private ScoringSystem mag;
  private MecanumWheelBase wheelBase;

  @Override
  public void runOpMode() {
    BallSequence wantedSequence = BallSequence.PGP; // the sequence we want to shoot

    // Initialize motors/servos/sensors
    DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
    DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
    DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
    DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

    DcMotorEx turretRotator = hardwareMap.get(DcMotorEx.class, "turretRotator");
    DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
    DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

    DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
    ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    // the following is temporary; TODO: remove once Auto code configures stuff
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    pinpoint.resetPosAndIMU(); // TODO: only calibrate IMU once Auto code configures stuff

    Servo turretHood = hardwareMap.get(Servo.class, "turretHood");
    Servo rawLifterServo1 = hardwareMap.get(Servo.class, "lifter1");
    Servo rawLifterServo2 = hardwareMap.get(Servo.class, "lifter2");
    Servo rawMagServo1 = hardwareMap.get(Servo.class, "magServo1");
    Servo rawMagServo2 = hardwareMap.get(Servo.class, "magServo2");

    AnalogInput magServoEncoder = hardwareMap.get(AnalogInput.class, "magServoEncoder");
    AnalogInput lifterServoEncoder = hardwareMap.get(AnalogInput.class, "rampServoEncoder");

    Axon magServo1 = new Axon(rawMagServo1, magServoEncoder);
    Axon magServo2 = new Axon(rawMagServo2, magServoEncoder);
    Axon lifterServo1 = new Axon(rawLifterServo1, lifterServoEncoder);
    Axon lifterServo2 = new Axon(rawLifterServo2, lifterServoEncoder);

    Turret turret = new Turret(flywheelMotor, turretRotator, turretHood);

    ActiveIntake intake = new ActiveIntake(intakeMotor);

    Spindex spindex =
        new Spindex(magServo1, magServo2, lifterServo1, lifterServo2, colorSensor, distanceSensor);

    Camera camera =
        new Camera(
            hardwareMap,
            "camera",
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0),
            AllianceColor.RED); // placeholder TODO: update

    ScoringSystem mag =
        new ScoringSystem(intake, turret, spindex, camera, wantedSequence, telemetry);
    mag.init(false); // initialize scoring systems | after auto, mag is empty
    mag.setTargetDistance(100); // PLACEHOLDER

    MecanumWheelBase wheelBase = new MecanumWheelBase(frontLeft, frontRight, backLeft, backRight);

    waitForStart();

    mag.start(); // start scoring systems up

    while (opModeIsActive()) {
      // get robot position
      pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
      Pose2D location = pinpoint.getPosition();

      // update scoring systems
      mag.update();

      // emergency eject
      if (gamepad1.bWasPressed()) {
        mag.emergencyEject(); // eject intake at full power for a short time
      }

      // shoot
      if (gamepad1.rightBumperWasPressed()) {
        mag.shootMag(wantedSequence); // shoot all balls in the mag, in a sequence if possible
      }

      // intake
      if (gamepad1.aWasPressed()) {
        mag.fillMagUnsorted(); // fill the mag with any three balls

      } else if (gamepad1.xWasPressed()) {
        mag.fillMagSorted(); // fill the mag with 1 green and 2 purple balls

      } else if (gamepad1.yWasPressed()) {
        mag.intakeBall(); // intake one ball into mag
      }

      // update wheelbase
      wheelBase.setRotation(location.getHeading(AngleUnit.DEGREES)); // for field-centric control
      wheelBase.setThrottle(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

      // update telemetry
      telemetry.update();
    }

    mag.stop(); // stop all powered movement in scoring systems
    wheelBase.stop(); // stop all powered movement in wheels
  }
}
