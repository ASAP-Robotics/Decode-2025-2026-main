package org.firstinspires.ftc.teamcode.types;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class VirtualRobot {
    public Pose2D position;
    public double angle;

    public VirtualRobot(Pose2D position, double angle){
        this.position = position;
        this.angle = angle;
    }
}
