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

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.setScoringPose;
//import org.firstinspires.ftc.teamcode.actions.shootAction;
import org.firstinspires.ftc.teamcode.actions.updateScoring;
import org.firstinspires.ftc.teamcode.actions.updateTelemetry;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.SimpleTimer;

/** class to contain the behavior of the robot in Auto, to avoid code duplication */
public class SimpleAuto extends CommonRobot {
    // stuff (variables, etc., see TeliOpRobot) goes here;
    private int flipx = 1; //flip over the y axis
    private int flipy = 1; //flip over the x axis
    private double rotate = 0;
    protected final Pose2d beginPose;
    protected MecanumDrive drive;

    public SimpleAuto(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hardwareMap, telemetry, allianceColor, true);
        beginPose = allianceColor.getAutoStartPosition();
        drive = new MecanumDrive(hardwareMap, beginPose);
    }

    public void init() {
        SimpleTimer backup = new SimpleTimer(2);
        backup.start();
        while (drive.localizer.getState() != GoBildaPinpointDriver.DeviceStatus.READY
                && !backup.isFinished()) {
            drive.localizer.update();
        }
        //scoringSystem.init(true, true);
    }

    public void initLoop() {
        //scoringSystem.initLoop();
    }

    public void start() {
        //scoringSystem.start(false, false); // start scoring systems up
        if (allianceColor == AllianceColor.BLUE) {
            //flipx = -1;
            rotate = Math.PI;
            flipy = -1;

        }


        Actions.runBlocking(
                new ParallelAction( // BIGGEST BOI
                        //new updateScoring(scoringSystem),
                        new updateTelemetry(telemetry),
                        new SequentialAction( // BIG BOI
                                new SequentialAction(new setScoringPose(scoringSystem, allianceColor)), // 1
                                new SequentialAction( // shoot 1
                                        drive
                                                .actionBuilder(beginPose)
                                                .splineToLinearHeading(
                                                        new Pose2d(
                                                                -7 * flipx, 31 * flipy, flipy * (Math.toRadians(90))),
                                                        flipy*(Math.PI / 4),
                                                        new TranslationalVelConstraint(250.0),
                                                        new ProfileAccelConstraint(-50, 180))
                                                .build()//,
                                        //new shootAction(scoringSystem)
                                ))));

    }

    public void stop() {
    }

    @Override
    public void loop() {
    }
}