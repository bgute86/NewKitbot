package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Simple on-the-fly (OTF) path generator for PathPlannerLib 2025+.
 *
 * Requires AutoBuilder.configure(...) to have been called in your drive subsystem.
 */
public class OTFPath {

    // max vel (m/s), max accel (m/s^2), max angular vel (rad/s), max angular accel (rad/s^2)
    private static final PathConstraints DEFAULT_CONSTRAINTS =
            new PathConstraints(3.0, 3.0, Math.PI * 2, Math.PI * 4);

    /**
     * Builds a path from start -> (optional midpoints) -> end.
     *
     * IMPORTANT: the Rotation2d on each Pose2d here is the DIRECTION OF TRAVEL
     * (where the path heads), not the robot's heading. The robot's final heading
     * is set separately by endPose's rotation via GoalEndState.
     */
    public static PathPlannerPath generate(
            Pose2d start, Pose2d end, PathConstraints constraints, Translation2d... mids) {

        List<Pose2d> poses = new ArrayList<>();

        // Point the start toward the first target so the path starts heading the right way
        Translation2d firstTarget = mids.length > 0 ? mids[0] : end.getTranslation();
        poses.add(new Pose2d(start.getTranslation(), headingTo(start.getTranslation(), firstTarget)));

        // Midpoints: each one faces the next point
        for (int i = 0; i < mids.length; i++) {
            Translation2d next = (i + 1 < mids.length) ? mids[i + 1] : end.getTranslation();
            poses.add(new Pose2d(mids[i], headingTo(mids[i], next)));
        }

        // End point: face the same direction we were traveling when we arrived
        Translation2d prev = poses.get(poses.size() - 1).getTranslation();
        poses.add(new Pose2d(end.getTranslation(), headingTo(prev, end.getTranslation())));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // ideal starting state (only needed for pre-planned paths)
                new GoalEndState(0.0, end.getRotation())); // stop at end, robot rotates to end pose heading

        path.preventFlipping = true; // we already built it in the correct field coordinates
        return path;
    }

    /** Overload using default constraints. */
    public static PathPlannerPath generate(Pose2d start, Pose2d end, Translation2d... mids) {
        return generate(start, end, DEFAULT_CONSTRAINTS, mids);
    }

    /**
     * Command that generates the path WHEN IT'S SCHEDULED (so it uses the robot's
     * current pose at that moment) and then follows it.
     */
    public static Command driveTo(Supplier<Pose2d> currentPose, Pose2d target, Translation2d... mids) {
        return Commands.defer(
                () -> {
                    Pose2d start = currentPose.get();

                    // Skip tiny paths, PathPlanner can't build a path between near-identical points
                    if (start.getTranslation().getDistance(target.getTranslation()) < 0.1) {
                        return Commands.none();
                    }
                    return AutoBuilder.followPath(generate(start, target, mids));
                },
                java.util.Set.of()); // AutoBuilder.followPath already requires your drive subsystem
    }

    /**
     * Even simpler alternative: let PathPlanner pathfind (and avoid obstacles in the
     * navgrid) instead of building a path yourself.
     */
    public static Command pathfindTo(Pose2d target) {
        return AutoBuilder.pathfindToPose(target, DEFAULT_CONSTRAINTS, 0.0);
    }

    private static Rotation2d headingTo(Translation2d from, Translation2d to) {
        return to.minus(from).getAngle();
    }
}