package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@Config
@Autonomous(name = "automotomonous ", group = "OpMode")
public class automotomonous extends OpMode {


    @Override
    public void init() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    @Override
    public void loop() {

    }
}
;