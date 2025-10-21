package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * AprilTagLocator --------------- Drop-in helper for FTC AprilTags using VisionPortal +
 * AprilTagProcessor.
 *
 * <p>How to use (in your OpMode): ---------------------------- Position camPos = new
 * Position(DistanceUnit.INCH, -5, 7, 12, 0); // <-- set for YOUR mount YawPitchRollAngles camOri =
 * new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
 *
 * <p>AprilTagLocator tags = new AprilTagLocator(hardwareMap, true, "Webcam 1", camPos, camOri);
 * tags.start(); // call in init() waitForStart(); while (opModeIsActive()) { tags.update(); // call
 * every loop if (tags.isTagVisible()) { telemetry.addData("ID", tags.getLastSeenId());
 * telemetry.addData("XYZ (in)", "%.1f %.1f %.1f", tags.getX(), tags.getY(), tags.getZ());
 * telemetry.addData("PRY (deg)", "%.1f %.1f %.1f", tags.getPitch(), tags.getRoll(), tags.getYaw());
 * } else { telemetry.addLine("No tag visible"); } telemetry.addData("Visible IDs",
 * tags.getLastVisibleIds()); telemetry.update(); } tags.stop(); // call when done
 *
 * <p>Filtering (optional): --------------------- tags.setAllowedIds(Set.of(1,2,3)); // only
 * consider these IDs (null/empty = any) tags.setSelectedId(2); // lock to a single ID (null to
 * clear) // Or cycle at runtime with your buttons, using getLastVisibleIds() to choose.
 */
public class CameraLowLevel {

  // --- Construction/config ---
  private final HardwareMap hardwareMap;
  private final boolean useWebcam;
  private final String webcamName; // ignored if using phone camera

  // Camera pose on robot (adjust for your specific mounting)
  private final Position cameraPosition; // inches
  private final YawPitchRollAngles cameraOrientation; // degrees

  // --- Vision objects ---
  private VisionPortal visionPortal;
  private AprilTagProcessor aprilTag;

  // --- Cached results ---
  private boolean tagVisible = false;
  private int lastSeenId = -1;
  private double x, y, z; // inches
  private double pitch, roll, yaw; // degrees

  // --- Filtering/selection ---
  private Set<Integer> allowedIds = null; // null/empty => allow ANY
  private Integer selectedId = null; // if set, only accept this ID
  private final List<Integer> lastVisibleIds = new ArrayList<>();

  public CameraLowLevel(
      HardwareMap hardwareMap,
      boolean useWebcam,
      String webcamName,
      Position cameraPosition,
      YawPitchRollAngles cameraOrientation) {
    this.hardwareMap = hardwareMap;
    this.useWebcam = useWebcam;
    this.webcamName = (webcamName == null || webcamName.isEmpty()) ? "Webcam 1" : webcamName;
    this.cameraPosition = cameraPosition;
    this.cameraOrientation = cameraOrientation;
  }

  /** Build processor/portal. Call in init(). */
  public void start() {
    aprilTag =
        new AprilTagProcessor.Builder()
            // .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            // .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build();

    VisionPortal.Builder builder = new VisionPortal.Builder();
    if (useWebcam) {
      builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
    } else {
      builder.setCamera(BuiltinCameraDirection.BACK);
    }
    builder.addProcessor(aprilTag);
    visionPortal = builder.build();
  }

  /** Stop/close camera resources. */
  public void stop() {
    if (visionPortal != null) {
      visionPortal.close();
      visionPortal = null;
    }
    aprilTag = null;
  }

  /** Optional: pause/resume streaming to save CPU. */
  public void pause() {
    if (visionPortal != null) visionPortal.stopStreaming();
  }

  public void resume() {
    if (visionPortal != null) visionPortal.resumeStreaming();
  }

  /** Accept only these IDs (null/empty = accept any). */
  public void setAllowedIds(Set<Integer> ids) {
    this.allowedIds = (ids == null || ids.isEmpty()) ? null : new HashSet<>(ids);
  }

  /** Accept only this single ID (null to clear). */
  public void setSelectedId(Integer id) {
    this.selectedId = id;
  }

  /** Clear all filters (see any tag). */
  public void clearFilters() {
    this.allowedIds = null;
    this.selectedId = null;
  }

  /** IDs visible this frame (after filtering). */
  public List<Integer> getLastVisibleIds() {
    return new ArrayList<>(lastVisibleIds);
  }

  /**
   * Call every loop. Updates visibility and pose cache. Selection priority: selectedId (if set) →
   * tag with metadata, nearest by range (if available) → first match.
   */
  public void update() {
    tagVisible = false;
    lastSeenId = -1;
    lastVisibleIds.clear();

    if (aprilTag == null) return;

    List<AprilTagDetection> detections = aprilTag.getDetections();
    if (detections == null || detections.isEmpty()) return;

    // Collect IDs that pass filters for UI/selection
    for (AprilTagDetection d : detections) {
      if (passesFilter(d)) lastVisibleIds.add(d.id);
    }
    if (lastVisibleIds.isEmpty()) return;

    // Choose best passing detection
    AprilTagDetection best = null;

    // 1) If locked to a specific ID, prefer it
    if (selectedId != null) {
      for (AprilTagDetection d : detections) {
        if (passesFilter(d) && d.id == selectedId) {
          best = d;
          break;
        }
      }
    }

    // 2) Otherwise, prefer ones with metadata and closer range (if available)
    if (best == null) {
      double bestScore = Double.POSITIVE_INFINITY; // lower is better
      for (AprilTagDetection d : detections) {
        if (!passesFilter(d)) continue;

        double score = 0.0;
        if (d.metadata == null) score += 1000.0; // prefer known field tags
        if (d.ftcPose != null) score += Math.abs(d.ftcPose.range); // nearer better

        if (score < bestScore) {
          bestScore = score;
          best = d;
        }
      }
    }

    if (best == null) return;

    // Cache pose from robotPose (XYZ inches, PRY degrees)
    x = best.robotPose.getPosition().x;
    y = best.robotPose.getPosition().y;
    z = best.robotPose.getPosition().z;

    pitch = best.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
    roll = best.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
    yaw = best.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

    lastSeenId = best.id;
    tagVisible = true;
  }

  // --- Internal helpers ---
  private boolean passesFilter(AprilTagDetection d) {
    if (d == null) return false;
    if (selectedId != null && d.id != selectedId) return false;
    if (allowedIds != null && !allowedIds.contains(d.id)) return false;
    return true;
  }

  // --- Getters for current pose state ---
  public boolean isTagVisible() {
    return tagVisible;
  }

  public int getLastSeenId() {
    return lastSeenId;
  }

  // Position (inches): +X right, +Y forward, +Z up (FTC field convention via robotPose)
  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getZ() {
    return z;
  }

  // Orientation (degrees): Pitch about +X, Roll about +Y, Yaw about +Z
  public double getPitch() {
    return pitch;
  }

  public double getRoll() {
    return roll;
  }

  public double getYaw() {
    return yaw;
  }
}
