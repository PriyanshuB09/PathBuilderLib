import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A CSP-made class utilizing PathPlanner AutoBuilder, allowing us to create paths, follow paths,
 * and get drive properties based on the robot mode.
 */
public final class PathBuilder {
  private static Drive drive;

  public static FollowPath.Builder pathBuilder;

  private static PathConstraints constraints =
      new PathConstraints(
          Constants.MaxVelocity * 0.8,
          Constants.MaxAcceleration * 0.8,
          Constants.MaxAngularVelocity * 0.8,
          Constants.MaxAngularAcceleration * 0.8);

  /**
   * A method to configure the PathBuilder class, setting it up with the Drivetrain instance.
   * Enables PathFinding and AutoBuilder.
   *
   * @param drivetrain
   */
  public static void configure(Drive drivetrain) {
    if (drive != null) return;
    drive = drivetrain;

    AutoBuilder.configure(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                Constants.DrivePID.getP(),
                Constants.DrivePID.getI(),
                Constants.DrivePID.getD()),
            new PIDConstants(
                Constants.AnglePID.getP(),
                Constants.AnglePID.getI(),
                Constants.AnglePID.getD())),
        Constants.RobotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  /**
   * A method that takes in a Translation2d that the robot will orient towards.
   *
   * @param wanted A Translation2d Supplier
   */
  public static void targetTranslation(Supplier<Translation2d> wanted) {
    targetRotation(() -> wanted.get().minus(drive.getPose().getTranslation()).getAngle());
  }

  /**
   * A method takes in a Rotation2d that the robot will orient towards
   *
   * @param wanted A Rotation2d Supplier
   */
  public static void targetRotation(Supplier<Rotation2d> wanted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Supplier<Rotation2d> rotationSupplier = wanted;
          Constants.AnglePID.enableContinuousInput(-Math.PI, Math.PI);

          Logger.recordOutput(
              "PathBuilder/Track Target Angle", rotationSupplier.get().getRadians());
          Logger.recordOutput(
              "PathBuilder/Track Current Angle", drive.getPose().getRotation().getRadians());

          double omega =
              Constants.DriveConstants.AnglePID.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians())
                  + Constants.DriveConstants.AnglePID.getSetpoint().velocity
                      * Constants.AngleFeedForward;

          if (Math.abs(drive.getRotation().getRadians() - rotationSupplier.get().getRadians())
                  < Constants.AngleTolerance.getRadians()
              && Constants.AnglePID.getSetpoint().velocity == 0.0) omega = 0.0;

          return omega;
        });
  }

  /*
   * Stops any current rotation tracking
   */
  public static void stopTarget() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  /**
   * @return get current PathConstraints for AutoBuilder
   */
  public static PathConstraints getConstraints() {
    return constraints;
  }

  /**
   * @param newConstraints the new PathConstraints for AutoBuilder
   */
  public static void setConstraints(PathConstraints newConstraints) {
    constraints = newConstraints;
  }

  public static class Target {
    public final Pose2d pose;
    public final double speedMultiplier;
    public final double rotationLeadMeters;
    public final double rotationSpread;

    public Target(Pose2d pose) {
      this(pose, 1.0, 0.0, 1.0);
    }

    public Target(Pose2d pose, double speedMultiplier) {
      this(pose, speedMultiplier, 0.0, 1.0);
    }

    public Target(Pose2d pose, double speedMultiplier, double rotationLeadMeters) {
      this(pose, speedMultiplier, rotationLeadMeters, 1.0);
    }

    public Target(
        Pose2d pose, double speedMultiplier, double rotationLeadMeters, double rotationSpread) {
      if (pose == null) throw new IllegalArgumentException("pose cannot be null");
      this.pose = pose;
      this.speedMultiplier = speedMultiplier;
      this.rotationLeadMeters = rotationLeadMeters;
      this.rotationSpread = rotationSpread;
    }
  }

  // Way too much work into this

  public static Command path(Target... targets) {
    if (targets == null || targets.length == 0) {
      throw new IllegalArgumentException("Must supply at least one Target");
    }

    final double rotIndexRadius = Constants.PathCreationTolerance;

    Pose2d[] poses = new Pose2d[targets.length];
    for (int i = 0; i < targets.length; i++) poses[i] = targets[i].pose;

    List<Pose2d> travelHeadingPoses = new ArrayList<>(poses.length);
    for (int i = 0; i < poses.length; i++) {
      Translation2d pos = poses[i].getTranslation();
      Rotation2d heading;
      if (poses.length == 1) {
        heading = poses[i].getRotation();
      } else if (i == 0) {
        Translation2d next = poses[i + 1].getTranslation();
        heading =
            safeHeading(next.getX() - pos.getX(), next.getY() - pos.getY(), poses[i].getRotation());
      } else if (i == poses.length - 1) {
        Translation2d prev = poses[i - 1].getTranslation();
        heading =
            safeHeading(pos.getX() - prev.getX(), pos.getY() - prev.getY(), poses[i].getRotation());
      } else {
        Translation2d prev = poses[i - 1].getTranslation();
        Translation2d next = poses[i + 1].getTranslation();
        heading =
            safeHeading(
                next.getX() - prev.getX(), next.getY() - prev.getY(), poses[i].getRotation());
      }
      travelHeadingPoses.add(new Pose2d(pos, heading));
    }

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(travelHeadingPoses);
    final int waypointSlots = Math.max(1, waypoints.size() - 1);

    List<RotationTarget> emptyRotationTargets = Collections.<RotationTarget>emptyList();
    List<PointTowardsZone> emptyPointTowards = Collections.<PointTowardsZone>emptyList();
    List<ConstraintsZone> emptyConstraintsZones = Collections.<ConstraintsZone>emptyList();
    List<EventMarker> emptyEventMarkers = Collections.<EventMarker>emptyList();

    PathConstraints globalConstraints = PathBuilder.getConstraints();

    GoalEndState tmpGoal = new GoalEndState(0.0, poses[poses.length - 1].getRotation());
    PathPlannerPath tempPath =
        new PathPlannerPath(
            waypoints,
            emptyRotationTargets,
            emptyPointTowards,
            emptyConstraintsZones,
            emptyEventMarkers,
            globalConstraints,
            null,
            tmpGoal,
            false);

    List<Pose2d> sampled = tempPath.getPathPoses();
    if (sampled == null || sampled.size() == 0) {
      List<RotationTarget> fallbackRT = new ArrayList<>();
      int n = poses.length;
      fallbackRT.add(new RotationTarget(0.0, poses[0].getRotation()));
      for (int i = 1; i < n; i++) {
        double pct = (n == 1) ? 0.0 : ((double) i) / (n - 1);
        if (!poses[i].getRotation().equals(poses[i - 1].getRotation())) {
          double waypointRelPos = pct * waypointSlots;
          fallbackRT.add(new RotationTarget(waypointRelPos, poses[i].getRotation()));
        }
      }
      GoalEndState finalGoal = new GoalEndState(0.0, poses[poses.length - 1].getRotation());
      PathPlannerPath fallback =
          new PathPlannerPath(
              waypoints,
              fallbackRT,
              emptyPointTowards,
              emptyConstraintsZones,
              emptyEventMarkers,
              globalConstraints,
              null,
              finalGoal,
              false);
      return AutoBuilder.followPath(fallback);
    }

    int m = sampled.size();
    double[] cum = new double[m];
    cum[0] = 0.0;
    for (int i = 1; i < m; i++) {
      double seg = sampled.get(i).getTranslation().getDistance(sampled.get(i - 1).getTranslation());
      cum[i] = cum[i - 1] + seg;
    }
    double totalLength = cum[m - 1];
    if (totalLength <= 1e-9) totalLength = 1e-9;
    int searchSegStart = 0;
    double lastArcAccepted = 0.0;
    int nTargets = targets.length;
    double[] targetArcs = new double[nTargets];

    for (int t = 0; t < nTargets; t++) {
      Translation2d targetTrans = targets[t].pose.getTranslation();

      if (sampled.size() == 1) {
        double d = sampled.get(0).getTranslation().getDistance(targetTrans);
        targetArcs[t] = (d <= rotIndexRadius) ? 0.0 : totalLength;
        if (targetArcs[t] < lastArcAccepted) targetArcs[t] = lastArcAccepted;
        lastArcAccepted = targetArcs[t];
        continue;
      }

      double bestDist = Double.POSITIVE_INFINITY;
      double bestArc = Double.POSITIVE_INFINITY;
      int bestSeg = Math.max(0, searchSegStart);

      for (int i = searchSegStart; i < sampled.size() - 1; i++) {
        Translation2d a = sampled.get(i).getTranslation();
        Translation2d b = sampled.get(i + 1).getTranslation();
        double ax = a.getX(), ay = a.getY(), bx = b.getX(), by = b.getY();
        double dx = bx - ax, dy = by - ay;
        double segLenSq = dx * dx + dy * dy;

        double u = 0.0;
        if (segLenSq > 1e-12) {
          double tx = targetTrans.getX() - ax;
          double ty = targetTrans.getY() - ay;
          u = (tx * dx + ty * dy) / segLenSq;
          if (u < 0.0) u = 0.0;
          else if (u > 1.0) u = 1.0;
        }

        double projX = ax + u * dx;
        double projY = ay + u * dy;
        double dist = Math.hypot(targetTrans.getX() - projX, targetTrans.getY() - projY);

        double segLen = Math.hypot(dx, dy);
        double arcAlong = cum[i] + (segLen * u);

        if (arcAlong + 1e-9 < lastArcAccepted) continue;

        if (dist < bestDist) {
          bestDist = dist;
          bestArc = arcAlong;
          bestSeg = i;
        }
      }

      if (bestDist == Double.POSITIVE_INFINITY) {
        double bestSampleDist = Double.POSITIVE_INFINITY;
        int bestSampleIdx = Math.min(searchSegStart, sampled.size() - 1);
        for (int i = searchSegStart; i < sampled.size(); i++) {
          double d = sampled.get(i).getTranslation().getDistance(targetTrans);
          if (d < bestSampleDist) {
            bestSampleDist = d;
            bestSampleIdx = i;
          }
        }
        int segIdx = Math.max(0, Math.min(sampled.size() - 2, bestSampleIdx - 1));
        Translation2d a = sampled.get(segIdx).getTranslation();
        Translation2d b = sampled.get(segIdx + 1).getTranslation();
        double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
        double segLenSq = dx * dx + dy * dy;
        double u = 0.0;
        if (segLenSq > 1e-12) {
          double tx = targetTrans.getX() - a.getX();
          double ty = targetTrans.getY() - a.getY();
          u = (tx * dx + ty * dy) / segLenSq;
          if (u < 0.0) u = 0.0;
          else if (u > 1.0) u = 1.0;
        }
        double segLen = Math.hypot(dx, dy);
        bestArc = cum[segIdx] + (segLen * u);
        bestSeg = segIdx;
      }

      if (bestArc < lastArcAccepted) bestArc = lastArcAccepted;
      if (bestArc > totalLength) bestArc = totalLength;
      targetArcs[t] = bestArc;
      lastArcAccepted = bestArc;
      searchSegStart = Math.min(bestSeg, sampled.size() - 2);
    }

    List<RotationTarget> rotationTargets = new ArrayList<>();
    rotationTargets.add(new RotationTarget(0.0, poses[0].getRotation()));
    Rotation2d lastRotation = poses[0].getRotation();
    final double ANGLE_TOL_RAD = Math.toRadians(0.1);
    final double PRE_OFFSET_ARC = Math.max(1e-3, totalLength * 1e-6);

    for (int t = 1; t < nTargets; t++) {
      double arc = targetArcs[t];
      Rotation2d desired = poses[t].getRotation();

      double angDiff = Math.abs(normalizeAngle(desired.getRadians() - lastRotation.getRadians()));
      if (angDiff <= ANGLE_TOL_RAD) {
        continue;
      }

      double leadMeters = targets[t].rotationLeadMeters;
      double rotationStartArc = arc - leadMeters;
      rotationStartArc = Math.max(0.0, Math.min(rotationStartArc, totalLength));

      double baseEndArc = (leadMeters >= 0.0) ? arc : (arc - leadMeters);
      baseEndArc = Math.max(0.0, Math.min(baseEndArc, totalLength));

      double baseWindow = Math.abs(baseEndArc - rotationStartArc);
      if (baseWindow < 1e-12) baseWindow = PRE_OFFSET_ARC;
      double spread = targets[t].rotationSpread;
      if (!Double.isFinite(spread) || spread <= 0.0) spread = 1.0;

      double spreadWindow = baseWindow * spread;
      double rotationEndArc = rotationStartArc + spreadWindow;
      rotationEndArc = Math.max(0.0, Math.min(rotationEndArc, totalLength));

      double preArc = rotationStartArc;
      double preWaypointPos = (preArc / totalLength) * waypointSlots;
      double finalWaypointPos = (rotationEndArc / totalLength) * waypointSlots;

      if (finalWaypointPos <= preWaypointPos + 1e-12) {
        finalWaypointPos = Math.min(waypointSlots, preWaypointPos + 1e-6);
        preWaypointPos = Math.max(0.0, finalWaypointPos - 1e-6);
      }

      rotationTargets.add(new RotationTarget(preWaypointPos, lastRotation));
      rotationTargets.add(new RotationTarget(finalWaypointPos, desired));
      lastRotation = desired;
    }

    double finalWaypointPos = waypointSlots;
    Rotation2d finalRot = poses[poses.length - 1].getRotation();
    if (Math.abs(normalizeAngle(finalRot.getRadians() - lastRotation.getRadians()))
        > ANGLE_TOL_RAD) {
      rotationTargets.add(new RotationTarget(finalWaypointPos, finalRot));
    }

    List<ConstraintsZone> constraintsZones = new ArrayList<>();
    final double ZONE_EPSILON = 1e-6;
    for (int i = 0; i < nTargets; i++) {
      if (Math.abs(targets[i].speedMultiplier - 1.0) <= 1e-9) continue;

      if (i > 0 && Math.abs(targets[i - 1].speedMultiplier - 1.0) > 1e-9) {
        continue;
      }

      int j = i + 1;
      while (j < nTargets && Math.abs(targets[j].speedMultiplier - 1.0) > 1e-9) j++;

      double startArc = targetArcs[i];
      double endArc = (j < nTargets) ? targetArcs[j] : totalLength;

      if (endArc - startArc < ZONE_EPSILON) {
        endArc = Math.min(totalLength, startArc + 1e-3);
        if (endArc - startArc < ZONE_EPSILON) {
          startArc = Math.max(0.0, startArc - 1e-3);
        }
      }

      double startFrac = startArc / totalLength;
      double endFrac = endArc / totalLength;

      double startWaypointPos = Math.max(0.0, Math.min(waypointSlots, startFrac * waypointSlots));
      double endWaypointPos = Math.max(0.0, Math.min(waypointSlots, endFrac * waypointSlots));

      double mult = targets[i].speedMultiplier;
      PathConstraints scaled =
          new PathConstraints(
              Math.max(0.0, globalConstraints.maxVelocityMPS() * mult),
              Math.max(0.0, globalConstraints.maxAccelerationMPSSq() * mult),
              Math.max(0.0, globalConstraints.maxAngularVelocityRadPerSec() * Math.max(1.0, mult)),
              Math.max(
                  0.0,
                  globalConstraints.maxAngularAccelerationRadPerSecSq() * Math.max(1.0, mult)));

      constraintsZones.add(new ConstraintsZone(startWaypointPos, endWaypointPos, scaled));
    }

    constraintsZones = mergeConstraintsZones(constraintsZones);

    GoalEndState finalGoal = new GoalEndState(0.0, finalRot);
    PathPlannerPath finalPath =
        new PathPlannerPath(
            waypoints,
            rotationTargets,
            emptyPointTowards,
            constraintsZones.isEmpty() ? emptyConstraintsZones : constraintsZones,
            emptyEventMarkers,
            globalConstraints,
            null,
            finalGoal,
            false);

    return AutoBuilder.followPath(finalPath);
  }

  private static List<ConstraintsZone> mergeConstraintsZones(List<ConstraintsZone> zones) {
    if (zones == null || zones.size() <= 1) return zones;
    zones.sort(Comparator.comparingDouble(ConstraintsZone::minPosition));
    List<ConstraintsZone> out = new ArrayList<>();
    ConstraintsZone cur = zones.get(0);
    for (int i = 1; i < zones.size(); i++) {
      ConstraintsZone next = zones.get(i);
      boolean sameConstr = Objects.equals(cur.constraints(), next.constraints());
      if (sameConstr && next.minPosition() <= cur.maxPosition() + 1e-9) {
        cur =
            new ConstraintsZone(
                cur.minPosition(),
                Math.max(cur.maxPosition(), next.maxPosition()),
                cur.constraints());
      } else {
        out.add(cur);
        cur = next;
      }
    }
    out.add(cur);
    return out;
  }

  private static double normalizeAngle(double a) {
    return Math.atan2(Math.sin(a), Math.cos(a));
  }

  private static Rotation2d safeHeading(double dx, double dy, Rotation2d fallback) {
    final double EPS = 1e-6;
    if (Math.abs(dx) < EPS && Math.abs(dy) < EPS) {
      return fallback != null ? fallback : Rotation2d.fromDegrees(0.0);
    }
    return new Rotation2d(dx, dy);
  }

  public static Command shootOnMove(DoubleSupplier RPM) {
    return Commands.none();
  }

  // My own auto triggers :) very simple Commands but maintains a uniform structure through the
  // syntax

  public static Command triggerWhenClose(
      Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(
            () -> drive.getPose().getTranslation().getDistance(location) <= distance)
        .andThen(runnable);
  }

  public static Command triggerWhenFar(Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(
            () -> drive.getPose().getTranslation().getDistance(location) > distance)
        .andThen(runnable);
  }

  public static Command triggerWhenTrue(BooleanSupplier condition, Command runnable) {
    return Commands.waitUntil(condition).andThen(runnable);
  }

  public static Command triggerWithDelay(double seconds, Command runnable) {
    return Commands.waitSeconds(seconds).andThen(runnable);
  }

  /**
   * Creates a path off of a goal Pose2d using Pathfinding
   *
   * @param endPose Pose2d
   * @return Command
   */
  public static Command createPath(Pose2d endPose) {
    return AutoBuilder.pathfindToPose(endPose, constraints, 0.0).finallyDo(() -> drive.stop());
  }

  /**
   * Merges into a path using Pathfinding
   *
   * @param knownPath PathPlannerPath
   * @return Command
   */
  public static Command mergeToPath(PathPlannerPath knownPath) {
    return AutoBuilder.pathfindThenFollowPath(knownPath, constraints)
        .beforeStarting(
            () -> Constants.DriveConstants.ANGLE_PID.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }
}
