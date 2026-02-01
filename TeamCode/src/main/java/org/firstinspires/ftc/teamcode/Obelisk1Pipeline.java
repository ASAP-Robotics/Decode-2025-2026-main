package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight Biggest AprilTag", group = "Vision")
public class Obelisk1Pipeline extends OpMode {

    private Limelight3A limelight;
    private final int[] apriltagID = {21,22,23};

    private boolean[] aprilTagInSight = {false, false, false};
    private double[] apriltagSize = {0.0, 0.0, 0.0};



    // Set this to whatever pipeline index you configured as "AprilTags"
    private static final int APRILTAG_PIPELINE = 5;
    private int findBiggest() {
        int biggest = 0;
        for (int i = 1; i < apriltagSize.length; i++) {
            if (apriltagSize[i] > apriltagSize[biggest]) biggest = i;
        }
        return (apriltagSize[biggest] > 0.0) ? biggest : -1;
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        for (int i = 0; i < apriltagSize.length; i++) {
            apriltagSize[i] = 0.0;
            aprilTagInSight[i] = false;
        }
    }

    @Override
    public void start() {
        limelight.start();
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();


        if (result == null) {
            telemetry.addLine("Result: null (no data yet)");
        } else{

            telemetry.addLine("Actual Pipeline: " + result.getPipelineIndex());
            telemetry.addLine("Staleness (ms): " + result.getStaleness());

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                if(tag.getFiducialId() == 21){
                    aprilTagInSight[0] = true;
                    apriltagSize[0] = tag.getTargetArea();
                }
                if(tag.getFiducialId() == 22){
                    aprilTagInSight[1] = true;
                    apriltagSize[1] = tag.getTargetArea();
                }
                if(tag.getFiducialId() == 23){
                    aprilTagInSight[2] = true;
                    apriltagSize[2] = tag.getTargetArea();
                }

            }
            int biggest = findBiggest();
            if (biggest != -1) {
                telemetry.addLine("apriltag " + apriltagID[biggest]);
            } else {
                telemetry.addLine("No tag sizes recorded yet");
            }
            telemetry.update();

        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
