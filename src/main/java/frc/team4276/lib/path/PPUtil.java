package frc.team4276.lib.path;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import java.util.ArrayList;
import java.util.List;

public class PPUtil {
  public static PathPlannerTrajectory mirrorLengthwise(PathPlannerTrajectory trajectory) {
    List<PathPlannerTrajectoryState> mirroredStates = new ArrayList<>();
    for (var state : trajectory.getStates()) {
      mirroredStates.add(mirrorLengthwise(state));
    }
    return new PathPlannerTrajectory(mirroredStates, trajectory.getEvents());
  }

  private static final double[] dummyList = {0.0, 0.0, 0.0, 0.0};

  public static PathPlannerTrajectoryState mirrorLengthwise(PathPlannerTrajectoryState state) {
    var flipped = new PathPlannerTrajectoryState();

    flipped.timeSeconds = state.timeSeconds;
    flipped.linearVelocity = state.linearVelocity;
    flipped.pose = PathUtil.mirrorLengthwise(state.pose);
    flipped.feedforwards =
        new DriveFeedforwards(dummyList, dummyList, dummyList, dummyList, dummyList);
    flipped.fieldSpeeds = PathUtil.mirrorLengthwise(state.fieldSpeeds);
    flipped.heading = PathUtil.mirrorLengthwise(state.heading);

    return flipped;
  }
}
