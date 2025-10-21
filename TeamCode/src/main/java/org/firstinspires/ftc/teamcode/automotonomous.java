package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheelBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringSystem;
import org.firstinspires.ftc.teamcode.hardware.Spindex;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.AprilTag;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.BallSequence;

@Autonomous(name = "automotomonous")
public class automotomonous extends LinearOpMode {
    private DcMotorEx frontLeft,
            frontRight,
            backLeft,
            backRight,
            flywheelMotor,
            intakeMotor,
            turretRotator;
    private Servo magServo, feeder, turretHood;
    private IMU imu;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Turret turret;
    private ActiveIntake intake;
    private Spindex spindex;
    private Camera camera;
    private ScoringSystem mag;
    private MecanumWheelBase wheelBase;
    private boolean xPrev = false; // for rising-edge detect
    private boolean xToggle = false; // the thing you're toggling
    private boolean aPrev = false;

    // this is a super long line of comments and stuff to get the automatic formating to actually do
    // something. This really is quite a lot of text. Wow. Super long. Crazy. The formating had better
    // work!


    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //dalens init stuff
        // Initialize motors/servos/sensors
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        // TODO: add turret motor configuration
        turretRotator = hardwareMap.get(DcMotorEx.class, "turretRotator");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        // TODO: add turret servo configuration
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        feeder = hardwareMap.get(Servo.class, "feeder");
        magServo = hardwareMap.get(Servo.class, "magServo");

        turret = new Turret(flywheelMotor, turretRotator, turretHood);
        intake = new ActiveIntake(intakeMotor);
        spindex = new Spindex(magServo, feeder, colorSensor, distanceSensor);
        camera = new Camera(AllianceColor.RED); // placeholder TODO: update

        mag = new ScoringSystem(intake, turret, spindex, camera, telemetry);
        mag.setTargetDistance(100); // PLACEHOLDER


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive
                                .actionBuilder(new Pose2d(-38, -53, Math.toRadians(0)))
                                .waitSeconds(3)
                                .splineTo(new Vector2d(-12, -47), Math.toRadians(45))
                                .splineToConstantHeading(new Vector2d(-40, -47), Math.toRadians(0))
                                .waitSeconds(1)
                                //there back shoot
                                .splineTo(new Vector2d(10, -47), Math.toRadians(-45))
                                .splineToConstantHeading(new Vector2d(11, -48), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-40, -47), Math.toRadians(0))
                                .waitSeconds(1)
                                //there back shoot
                                //^ chunk 1 48 points if sorted.
                                .splineTo(new Vector2d(1,-47), Math.toRadians(0))
                                .strafeTo(new Vector2d(1, -55))
                                .waitSeconds(4)
                                //hit gate
                                .strafeTo(new Vector2d(1, -53))
                                .splineToConstantHeading(new Vector2d(25, -50), Math.toRadians(0))
                                .splineTo(new Vector2d(34, -46), Math.toRadians(0))
                                //leave and shoot
                                .splineToConstantHeading(new Vector2d(57, -22), Math.toRadians(270))
                                .waitSeconds(1)
                                //load shoot
                                .splineTo(new Vector2d(57, -53), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(57, -22), Math.toRadians(270))
                                .waitSeconds(1)
                                //load shoot
                                .splineTo(new Vector2d(57, -53), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(57, -22), Math.toRadians(270))
                                .waitSeconds(1)
                                .build()
                ));
    }
}
