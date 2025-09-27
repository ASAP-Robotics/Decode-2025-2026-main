package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Full Power", group = "Test")
public class MotorFullPower extends LinearOpMode {

    private DcMotor testMotor;

    @Override
    public void runOpMode() {
        // Map your motor (make sure the config name matches RC app)
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        telemetry.addLine("Motor Full Power ready!");
        telemetry.update();

        waitForStart();

        // Run motor full power until stop is pressed
        while (opModeIsActive()) {
            testMotor.setPower(1.0); // Full forward power

            telemetry.addData("Motor Power", "100%");
            telemetry.update();
        }

        // Stop motor when OpMode ends
        testMotor.setPower(0);
    }
}


