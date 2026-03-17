# Extending PathPlannerLib with Smarter Path Control

Previously, I posted about CSPPathing, our lightweight alternative to other forms of pathfinding. But after using it for a while, I noticed that tuning our paths or slightly fixing them was really hard, especially with our team stepping away from using the PathPlanner GUI. Oftentimes, I'd write a fix for a certain section of our autonomous routine, and the robot wouldn't obey that properly, so I decided to rethink how our team creates paths. Notice, this is not pathfinding. 

Using PathBuilder's AutoBuilder by passing in a PathPlannerPath is super useful, especially when you aren't dealing with having to manually make each autonomous in the GUI. However, I found PathPlannerLib was more for teams that already were using the GUI alongside the library. Classes like `RotationTarget` and `ConstraintZone` took in fractions of the entire path, which I found challenging to do without a lot of trial and error. 

I created a library built on top of PathPlannerLib that allows for more control on how the robot moves. There are a few methods in PathBuilder apart from the main pathing that we use on our team. <br><br>

First, `configure()`. It uses our team's constants to configure AutoBuilder. Call this on robot initialization.<br><br>

Second, `targetTranslation(Supplier<Translation2d>)` and `targetRotation(Supplier<Rotation2d>)`. Both of these override the robot's current rotation controller and orients the robot during motion towards a target translation or rotation. They're just methods, not commands, so use this: `Commands.runOnce(() -> PathBuilder.targetTranslation(() -> new Translation2d(0, 0)))`. You can stop any targeting overrides by running this: `PathBuilder.stopTarget()`.

Third, `getConstraints()` and `setConstraints(PathConstraints)`. If you want to change the constraints before the next path is run, you can change them through these helper methods.

Fourth, `triggerWhenTrue()`, `triggerWhenFar()`, and `triggerWhenClose()`. Instead of event markers, I found it better to have parallel commands running alongside the path that use these triggers to take effect when the robot goes through these points.

Fifth, `createPath(Pose2d)` and `mergeToPath(PathPlannerPath)`. These are pulled from AutoBuilder's methods, and I don't have the logic in yet, but theoretically, upon robot collision, the robot can still merge back into the path it was following.

