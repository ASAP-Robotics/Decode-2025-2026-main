package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Conversion factors for the GoBILDA linear slide
@Autonomous(name = "BasketAuto")
public class BasketAuto extends LinearOpMode {

  private static final double TICKS_PER_REV =
      537.6; // Encoder ticks per revolution (GoBILDA 5202 motor)
  private static final double LEAD_SCREW_TRAVEL =
      4.724410; // Distance the slide travels per motor revolution (in inches)
  private static final double TICKS_PER_INCH = TICKS_PER_REV / LEAD_SCREW_TRAVEL;

  double targetInchesUpDown = 35;
  double targetInchesUp = 2;

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    // sensors
    ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    // SLIDES:

    // Intake
    DcMotor rightIntakeSlide = hardwareMap.get(DcMotor.class, "rightIntakeSlide");
    DcMotor leftIntakeSlide = hardwareMap.get(DcMotor.class, "leftIntakeSlide");
    leftIntakeSlide.setDirection(DcMotor.Direction.REVERSE);

    // updown slides
    DcMotor rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
    DcMotor leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
    leftSlide.setDirection(DcMotor.Direction.REVERSE);

    // sensors
    TouchSensor rightIntakeLimitSwitch =
        hardwareMap.get(TouchSensor.class, "rightIntakeLimitSwitch");
    TouchSensor leftIntakeLimitSwitch = hardwareMap.get(TouchSensor.class, "leftIntakeLimitSwitch");
    TouchSensor upDownLimitSwitch = hardwareMap.get(TouchSensor.class, "upDownLimitSwitch");

    // SERVOS

    Servo clawFlipper = hardwareMap.get(Servo.class, "clawFlipper");
    Servo outputClaw = hardwareMap.get(Servo.class, "outputClaw");
    Servo intakeArmLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
    Servo intakeArmRight = hardwareMap.get(Servo.class, "intakeArmRight");
    Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
    Servo clawRotater = hardwareMap.get(Servo.class, "clawRotater");
    clawFlipper.setDirection(Servo.Direction.REVERSE);
    intakeArmLeft.setDirection(Servo.Direction.REVERSE);
    clawFlipper.setPosition(.59); // grabbing off wall and speciman scoreing
    outputClaw.setPosition(.48);
    intakeClaw.setPosition(.24); // open
    intakeArmLeft.setPosition(.25); // .22
    intakeArmRight.setPosition(.25);
    clawRotater.setPosition(.25);

    waitForStart();

    Actions.runBlocking(
        new ParallelAction(
            // SplineTo action
            drive
                .actionBuilder(new Pose2d(0, 0, 0 * Math.PI / 4))
                .splineToLinearHeading(
                    new Pose2d(10, 20, Math.toRadians(335)),
                    Math.PI / 4,
                    new TranslationalVelConstraint(12.0)) // Drive too basket for first drop
                .build(),

            // extendSlide action
            new extendSlide(leftSlide, rightSlide, intakeClaw, clawFlipper, 3050) // 1600,
            ));
}
