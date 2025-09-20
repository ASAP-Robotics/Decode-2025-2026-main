package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ball Sorter Mag (No Sleep)", group = "Sensor")
public class BallSorterMag extends LinearOpMode {

    private ColorSensor colorSensor;
    private Servo magServo;

    // 3-slot magazine
    private String[] slots = new String[3];
    private int currentSlot = 0;

    // Servo positions for the 3 slots
    private final double[] positions = {0.0, 0.33, 0.66}; // adjust to match hardware

    // Timing control
    private ElapsedTime runtime = new ElapsedTime();
    private double lastMoveTime = 0;
    private final double MOVE_DELAY = 400; // ms for servo to finish moving

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        magServo = hardwareMap.get(Servo.class, "magServo");

        // Start at slot 0
        magServo.setPosition(positions[currentSlot]);

        telemetry.addLine("Ball sorter ready!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Only check sensor if servo has finished last move
            if (runtime.milliseconds() - lastMoveTime > MOVE_DELAY) {
                int red   = colorSensor.red();
                int green = colorSensor.green();
                int blue  = colorSensor.blue();
                int total = red + green + blue;

                if (total > 50) { // ignore if nothing is seen
                    double rNorm = (double) red / total;
                    double gNorm = (double) green / total;
                    double bNorm = (double) blue / total;

                    String detected = null;

                    if (rNorm > 0.3 && bNorm > 0.3 && gNorm < 0.25) {
                        detected = "Purple";
                    } else if (gNorm > 0.4 && rNorm < 0.3 && bNorm < 0.3) {
                        detected = "Green";
                    }

                    if (detected != null) {
                        // Save detected color in current slot
                        slots[currentSlot] = detected;

                        // Move servo to next slot
                        currentSlot = (currentSlot + 1) % 3;
                        magServo.setPosition(positions[currentSlot]);

                        // Record move time
                        lastMoveTime = runtime.milliseconds();
                    }

                    telemetry.addData("Detected", detected);
                }
            }

            // Show magazine contents
            telemetry.addData("Slot 0", slots[0]);
            telemetry.addData("Slot 1", slots[1]);
            telemetry.addData("Slot 2", slots[2]);
            telemetry.addData("Current slot index", currentSlot);
            telemetry.update();
        }
    }
}

