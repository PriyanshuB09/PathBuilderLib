# Extending PathPlannerLib with Smarter Path Control

Previously, I posted about CSPPathing, our lightweight alternative to other forms of pathfinding. But after using it for a while, I noticed that tuning our paths or slightly fixing them was really hard, especially with our team stepping away from using the PathPlanner GUI. Oftentimes, I'd write a fix for a certain section of our autonomous, and the robot wouldn't obey that properly, so I decided to rethink how our team creates paths. Notice, this is not pathfinding. 

Using PathBuilder's AutoBuilder by passing in a PathPlannerPath is super useful, especially when you aren't dealing with having to manually make each autonomous in the GUI. However, I found PathPlannerLib was more for teams that already were using the GUI alongside the library. Classes like RotationTarget and ConstraintZone took in fractions of the entire path, which I found challenging to do without a lot of trial and error. 


