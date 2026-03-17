# Extending PathPlannerLib with Smarter Path Control

There are a few methods in PathBuilder apart from the main pathing that we use on our team. <br><br>

First, `PathBuilder.configure()`. It uses our team's constants to configure AutoBuilder. Call this on robot initialization.

Second, `PathBuilder.targetTranslation(Supplier<Translation2d>)` and `PathBuilder.targetRotation(Supplier<Rotation2d>)`. Both of these override the robot's current rotation controller and orients the robot during motion towards a target translation or rotation. They're just methods, not commands, so use this: `Commands.runOnce(() -> PathBuilder.targetTranslation(() -> new Translation2d(0, 0)))`. You can stop any targeting overrides by running this: `PathBuilder.stopTarget()`.

Third, `PathBuilder.getConstraints()` and `PathBuilder.setConstraints(PathConstraints)`. If you want to change the constraints before the next path is run, you can change them through these helper methods.

Fourth, `PathBuilder.triggerWhenTrue()`, `PathBuilder.triggerWhenFar()`, and `PathBuilder.triggerWhenClose()`. Instead of event markers, I found it better to have parallel commands running alongside the path that use these triggers to take effect when the robot goes through these points.

Fifth, `PathBuilder.createPath(Pose2d)` and `PathBuilder.mergeToPath(PathPlannerPath)`. These are pulled from AutoBuilder's methods, and I don't have the logic in yet, but theoretically, upon robot collision, the robot can still merge back into the path it was following.


The main pathing comes from just two things in PathBuilder. The nested class `PathBuilder.Target` and the method, `PathBuilder.path(PathBuilder.Target...)`, which returns a Path Following Command. Below is an example. 

```java
PathBuilder.path(
  new PathBuilder.Target(targetPose, speedMultiplier, rotationOffset, rotationSpread),
  new PathBuilder.Target(targetPose1, speedMultiplier1, rotationOffset1, rotationSpread1),
);
```
The target pose is a Pose2d, with the intended robot translation and rotation at that point. The speed multiplier is a scale factor on the robot's max velocity and max acceleration. The rotation offset is a distance in meters, where a positive value starts the rotation earlier in the path than the target pose, 0 starts the rotation immediately at the target pose, and a negative value starts the rotation later in the path. The rotation spread is a scale factor on how long the robot takes to rotate to the target's rotation. 

