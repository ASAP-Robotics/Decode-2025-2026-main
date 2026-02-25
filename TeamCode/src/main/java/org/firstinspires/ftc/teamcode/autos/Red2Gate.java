package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.types.AllianceColor;

@Autonomous(name = "Red Auto")
public class Red2Gate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            // SimpleAuto robot = new SimpleAuto(hardwareMap, telemetry, AllianceColor.RED);
            AutoRobot robot =
                    new AutoRobot(hardwareMap, telemetry, AllianceColor.RED, AutoRobot.paths.ClOSE15_2GATE);

            robot.init();

            while (opModeInInit()) {
                robot.initLoop();
            }

            waitForStart();

            robot.start();

            while (opModeIsActive()) {
                robot.loop();
            }
        }
    }
}
