# PathBuilder: A CSP Autonomous Library

## Documentation

PathBuilder has grown significantly after our first competition, the PCH Columbus Qualifier Event. The library has changed to be more intuitive and trustworthy. That meaning, it is more deterministic and customizable, two necessary aspects of any autonomous.

To configure PathBuilder, here are the two methods you can have. If you have the AdvantageKit template code, it is easy to set up.

```java
PathBuilder.configureDrive(
      boolean logged,
      double angleFF,
      Rotation2d angleTol,
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> setPose,
      Supplier<ChassisSpeeds> getChassisSpeeds,
      Runnable stopDrive,
      Consumer<ChassisSpeeds> runVelocity,
      ProfiledPIDController driveController,
      ProfiledPIDController angleController,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlip,
      Drive drive
);

PathBuilder.configurePathing(
    double pathCreationTolMeters, 
    double rotationTolRadians, 
    double followRotationMeters
);
```

If you don't have these methods, I highly recommend having them available for any autonomous code. Also, for `shouldFlip`, you can pass this in:
```java
() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
```

To create a path, you can use the `PathBuilder.path()` method, which returns a path following command.

To create different poses that define motion and events, you use the `PathBuilder.Target` class.

Here is how to implement them together.

```java
PathBuilder.path(
    new PathBuilder.Target(pose).withSpeed(0.2),
    new PathBuilder.Target(pose)
);
```

To build a `Target` you can add customizable chained methods on it. You can as many as you want to the end of a Target

```java
.withSpeed(scaleFactor)
.withRotationSpeed(scaleFactor)
.withRotationLead(rotationBeforeMeters)
.withRotationSpread(scaleFactor)
.withHeading(headingAngle)
.withTangentWeight(scaleFactor) // The smaller the value, the bigger the curve, if the value is negative, it splines the other way
.withTolerance(meters) // Prevents tiny changes in Target poses


// There are a few more, which are a little more complex

.onlyIf(BooleanSupplier ifTrue)

.withCommand(Command command) // Command that runs at that point


// There are 3 main Rotation Modes
// PathBuilder.Target.RotationMode.LINEAR <-- Default, with normal rotation
// PathBuilder.Target.RotationMode.SNAP <-- doesn't rotation at all until the Target pose
// PathBuilder.Target.RotationMode.FOLLOW <-- rotates towards the direction of motion

.withRotationMode(RotationMode)

```


There are a few more methods. 

```java
// Runs when robot gets close to the point
PathBuilder.triggerWhenClose(Translation2d location, double distanceMeters, Command command);

// Runs when the robot gets far from the point
PathBuilder.triggerWhenFar(Translation2d location, double distanceMeters, Command command);

// Uses PathPlanner's pathfinding to rejoin a path

PathBuilder.mergeToPath(PathPlannerPath path);

```
