package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

// Conversion factors for the GoBILDA linear slide
@Autonomous(name = "automotomonous")
//CHANGE ALL OF THE INIT STUFF CUZ IT WONT WORK!!!
public class automotomonous extends OpMode {
  
 static int state;
 public static String endState = "default";
  
@override
  public void init(); {
  private static final double TICKS_PER_REV =
      537.6; // Encoder ticks per revolution (GoBILDA 5202 motor)
  private static final double LEAD_SCREW_TRAVEL =
      4.724410; // Distance the slide travels per motor revolution (in inches)
  private static final double TICKS_PER_INCH = TICKS_PER_REV / LEAD_SCREW_TRAVEL;

  public double targetX;

  public double targetY;

  public double targetAngle;
  public double inchesPerPixel = 0.01015625;
  public double xMiddleScreen = 320;
  public double yMiddleScreen = 240;
  // OpenCV processing
  private ColorBlobLocatorProcessor colorLocator;
  private VisionPortal portal;

  boolean autoPickUp = false;
  public boolean sampleSeen = false;

  public String slidePositionUpDown = "in";
  public String slidePositionIntakeLeft = "in";
  double targetInchesUpDown = 35;
  double targetInchesUp = 2;

  double finalTargetX;
  double finalTargetY;
  
  state = 0; //initial state
}
  
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

    waitforstart();

    telemetry.addData("cur_state", state);
    
    switch(state) {   
      case 0: 
        //first section
        myBot.runAction(
        myBot
            .getDrive()
            .actionBuilder(new Pose2d(61.5, 30, Math.toRadians(180)))
            .splineTo(new Vector2d(-20, -20), Math.toRadians(220))
            .waitSeconds(3)
             .build());
        
        endState = "onto1"
        if(endState == "onto1") {
          state = 1;
        }
        
      case 1:
        //second section
        myBot.runAction(
        myBot
            .getDrive()
            .splineTo(new Vector2d(48, 47), Math.toRadians(180))
            .splineTo(new Vector2d(40, 47), Math.toRadians(180)) // i want this to be slower
            .splineTo(new Vector2d(-20, -20), Math.toRadians(220))
            .waitSeconds(3)
             .build());
        
        endState = "onto2"
          if(endState == "onto2") {
          state = 2;
        } 
        
      case 2:
        //third section
        myBot.runAction(
        myBot
            .getDrive()
            .splineTo(new Vector2d(20, 47), Math.toRadians(180))
            .splineTo(new Vector2d(12, 47), Math.toRadians(180))
            .splineTo(new Vector2d(-20, -20), Math.toRadians(220))
            .waitSeconds(3)
             .build());

        }  
}
