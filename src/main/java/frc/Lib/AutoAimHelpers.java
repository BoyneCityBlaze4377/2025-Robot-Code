package frc.Lib;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAimConstants.ReefWaypoint;

/** Add your docs here. */
public class AutoAimHelpers {
    public static double getEstimatedStationAngle(double angle) {
    double prevError = 180;
    double selectedAngle = 0;

    for (int i = 0; i < AutoAimConstants.reefStationAngles.length; i++) {
      double error = Math.abs(angle - AutoAimConstants.reefStationAngles[i]);
      if (error < prevError){
        prevError = error;
        selectedAngle = AutoAimConstants.reefStationAngles[i];
      }
    }
    return selectedAngle;
  }

  public static boolean pathIsInReef(AdvancedPose2D pose1, AdvancedPose2D pose2, Alliance alliance) {
    double initialX = pose1.getX();
    double initialY = pose1.getY();
    double distance = 0;
    double intervals = Math.floor(pose1.getDistance(pose2) / AutoAimConstants.inReefCalculationInterval);
    boolean inReef = false;
    double theta = Math.atan((pose2.getY() - pose1.getY()) / (pose2.getX() - pose1.getX()));
    boolean isBlue = alliance == Alliance.Blue;

    for (int i = 0; i <= intervals; i++) {
      distance = i * AutoAimConstants.inReefCalculationInterval;
      double x = initialX + distance * Math.cos(theta);
      double y = initialY + distance * Math.sin(theta);

      double xMax = isBlue ? AutoAimConstants.blueReefWaypointBoundMaxX : AutoAimConstants.redReefWaypointBoundMaxX;
      double xMin = isBlue ? AutoAimConstants.blueReefWaypointBoundMinX : AutoAimConstants.redReefWaypointBoundMinX;

      if (y < AutoAimConstants.reefWaypointBoundMaxY && y > AutoAimConstants.reefWaypointBoundMinY) {
        if (y < AutoAimConstants.reefWaypointStaticDisMinY || y > AutoAimConstants.reefWaypointBoundMaxY) {
            xMax = (isBlue ? FieldConstants.blueReefCenterPos.getX() : FieldConstants.blueReefCenterPos.horizontallyFlip().getX())
                    + (AutoAimConstants.reefWaypointBoundMaxY - y) * Math.tan(Units.degreesToRadians(60));
            xMin = (isBlue ? FieldConstants.blueReefCenterPos.getX() : FieldConstants.blueReefCenterPos.horizontallyFlip().getX())
                    - (AutoAimConstants.reefWaypointBoundMaxY - y) * Math.tan(Units.degreesToRadians(60));
        } else {
          xMax = isBlue ? AutoAimConstants.blueReefWaypointBoundMaxX : AutoAimConstants.redReefWaypointBoundMaxX;
          xMin = isBlue ? AutoAimConstants.blueReefWaypointBoundMinX : AutoAimConstants.redReefWaypointBoundMinX;
        }
        inReef = x <= xMax && x >= xMin;
      } else {
        inReef = false;
      }

      if (inReef) break;
    }
    return inReef;
  }

  public static ReefWaypoint[] getNearestTwoWaypoints(AdvancedPose2D currentPose, Alliance alliance) {
    ReefWaypoint closest = ReefWaypoint.blank;
    double prevDis = 200;
    double distance = 0;
    boolean isBlue = alliance == Alliance.Blue;
    HashMap<ReefWaypoint, AdvancedPose2D> reefWaypoints = isBlue ? AutoAimConstants.blueReefWaypoints : AutoAimConstants.redReefWaypoints;

    for (int i = 0; i < AutoAimConstants.waypointArray.length; i++) {
        ReefWaypoint waypoint = AutoAimConstants.waypointArray[i];
        distance = currentPose.getDistance(reefWaypoints.get(waypoint));
        if (distance < prevDis) {
            prevDis = distance;
            closest = waypoint;
        }    
    }

    ReefWaypoint closest2 = ReefWaypoint.blank;
    double prevDis2 = 200;
    double distance2 = 0;

    for (int i = 0; i < AutoAimConstants.waypointArray.length; i++) {
        ReefWaypoint waypoint2 = AutoAimConstants.waypointArray[i];
        distance2 = currentPose.getDistance(reefWaypoints.get(waypoint2));

        if ((distance2 < prevDis2) && (waypoint2 != closest)) {
            prevDis2 = distance2;
            closest2 = waypoint2;
        }    
    }

    ReefWaypoint[] closests = new ReefWaypoint[] {closest, closest2};
    String[] string = new String[2];

    for (int i = 0; i < 2; i++) {
      string[i] = closests[i].toString();
    }

    SmartDashboard.putStringArray("closests", string);

    return closests;
  }

  public static double calcPathAroundReefDistance(AdvancedPose2D firstPose, AdvancedPose2D finalPose, 
                                                  ReefWaypoint firstWaypoint, ReefWaypoint exemptedWaypoint, Alliance alliance) {
    boolean isBlue = alliance == Alliance.Blue;
    HashMap<ReefWaypoint, AdvancedPose2D> reefWaypoints = isBlue ? AutoAimConstants.blueReefWaypoints : AutoAimConstants.redReefWaypoints;
    AdvancedPose2D prevWaypoint = reefWaypoints.get(exemptedWaypoint);
    double distance = 0;
    AdvancedPose2D tempPose = firstPose;
    AdvancedPose2D nextPose = reefWaypoints.get(firstWaypoint);

    while (pathIsInReef(tempPose, nextPose, alliance)) {
      ReefWaypoint[] closest = getNearestTwoWaypoints(tempPose, alliance);
      AdvancedPose2D instancePose = tempPose;
      distance += tempPose.getDistance(nextPose);
      tempPose = nextPose;
      for (int i = 0; i < closest.length; i++) {
        if (reefWaypoints.get(closest[i]) != prevWaypoint) {
          prevWaypoint = instancePose;
          nextPose = reefWaypoints.get(closest[i]);
          break;
        }
      }
    }

    distance += tempPose.getDistance(finalPose);
    return distance;
  }

  public static ArrayList<AdvancedPose2D> getOptimalPath(AdvancedPose2D currentPose, AdvancedPose2D desiredPose, Alliance alliance) {
    HashMap<ReefWaypoint, AdvancedPose2D> reefWaypoints = alliance == Alliance.Blue ? AutoAimConstants.blueReefWaypoints : 
                                                          AutoAimConstants.redReefWaypoints;
    ReefWaypoint option1 = getNearestTwoWaypoints(currentPose, alliance)[0];
    ReefWaypoint option2 = getNearestTwoWaypoints(currentPose, alliance)[1];

    SmartDashboard.putString("OP1", option1.toString());
    SmartDashboard.putString("OP2", option2.toString());

    ReefWaypoint selectedWaypoint = calcPathAroundReefDistance(currentPose, desiredPose, option1, option2, alliance) < 
                                    calcPathAroundReefDistance(currentPose, desiredPose, option2, option1, alliance) 
                                    ? option2 : option1;
    
    ArrayList<AdvancedPose2D> optimalPath = new ArrayList<AdvancedPose2D>();
    optimalPath.add(reefWaypoints.get(selectedWaypoint));

    SmartDashboard.putNumber("Dis2", calcPathAroundReefDistance(currentPose, desiredPose, option1, option2, alliance));
    SmartDashboard.putNumber("Dis1", calcPathAroundReefDistance(currentPose, desiredPose, option2, option1, alliance));

    AdvancedPose2D tempPose = currentPose;
    AdvancedPose2D prevPose = reefWaypoints.get(ReefWaypoint.blank);
    AdvancedPose2D nextPose = reefWaypoints.get(selectedWaypoint);

    while (pathIsInReef(tempPose, nextPose, alliance)) {
      ReefWaypoint[] closest = getNearestTwoWaypoints(tempPose, alliance);
      AdvancedPose2D instancePose = tempPose;
      tempPose = nextPose;
      for (int i = 0; i < closest.length; i++) {
        if (reefWaypoints.get(closest[i]) != prevPose) {
          prevPose = instancePose;
          nextPose = reefWaypoints.get(closest[i]);
          break;
        }
      }

      optimalPath.add(nextPose);
    }

    optimalPath.add(desiredPose);

    SmartDashboard.putString("OPTIMAL", optimalPath.toString());
    return optimalPath;
  }
}
