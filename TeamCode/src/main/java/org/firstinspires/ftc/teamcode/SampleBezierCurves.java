/*
Bezier Curve PedroPathing AUTONOMOUS SAMPLE

This is a sample of an autonomous which uses PedroPathing. SampleBezierCurves.java serves as a
reference for building poses, control points, Bezier Curves, and Bezier Lines. If you are looking for
a guide for just Bezier Lines which show the basics, refer to SampleBezierCurves. As later versions of
PedroPathing come, you may have to update this and migrate to the later version through the official PedroPathing
website.

NOTE: Lines 58-73 are the ones that show Bezier Curves, everything else is the same as SampleAutoPathing.java

Initial Creation: 2/23/26 --Arthur
Latest Update:
*/

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SampleBezierCurves extends OpMode {
    // The brain of the whole process, handles the path building and updates movement pattern
    private Follower follower;
    // Create both timers
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }

    // Variable that holds the current path running
    PathState pathState;

    // Create all poses for paths
    private final Pose startPose = new Pose(20.386209877877445, 122.39783853885227, Math.toRadians(138));
    private final Pose shootPose = new Pose(46.415043769588245, 96.90020533880903, Math.toRadians(138));
    private final Pose endPose = new Pose(63.76759969739543, 105.75355019993515, Math.toRadians(90));

    // Create all paths
    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    // Builds all paths
    public void buildPaths() {
        // Control Points
        Pose control1 = new Pose(30.0, 140.0, 0);
        Pose control2 = new Pose(50.0, 80.0, 0);

        driveStartPosShootPos = follower.pathBuilder()
                // Pass the Poses directly into the BezierCurve
                .addPath(new BezierCurve(startPose, control1, control2, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    // Handles the order of movement
    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset the timer & make new state
                break;
            case SHOOT_PRELOAD:
                // check is follower done it's path?
                // and check that 5 seconds has elapsed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                // all done!
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all Paths");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        // Start opModeTimer when start button is clicked
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        // Run the follower
        follower.update();
        statePathUpdate();

        // Telemetry Data (Current Path, x, y, heading, Path Time)
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
