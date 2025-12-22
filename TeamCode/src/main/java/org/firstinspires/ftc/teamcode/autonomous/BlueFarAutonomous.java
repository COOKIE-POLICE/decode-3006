package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Preferences;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodLifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakerSubsystem;

@Autonomous
public class BlueFarAutonomous extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private LauncherSubsystem launcherSubsystem;
    private HoodLifterSubsystem hoodLifterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private OuttakerSubsystem outtakerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    // Poses

    private PathChain scorePreload, grabOneStart, grabOneEnd, grabOneLaunch, grabTwoStart, grabTwoEnd, grabTwoLaunch, grabThreeStart, grabThreeEnd, grabThreeLaunch;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(Preferences.Poses.BLUE_FAR_START_POSE, Preferences.Poses.BLUE_LAUNCH_POSE))
                .setLinearHeadingInterpolation(Preferences.Poses.BLUE_FAR_START_POSE.getHeading(), Preferences.Poses.BLUE_LAUNCH_POSE.getHeading())
                .build();

        grabOneStart = follower.pathBuilder()
                .addPath(new BezierLine(Preferences.Poses.BLUE_LAUNCH_POSE, Preferences.Poses.BLUE_GRAB_POSE_ONE_START))
                .setLinearHeadingInterpolation(Preferences.Poses.BLUE_LAUNCH_POSE.getHeading(), Preferences.Poses.BLUE_GRAB_POSE_ONE_START.getHeading())
                .build();
        grabOneEnd = follower.pathBuilder()
                .addPath(new BezierLine(Preferences.Poses.BLUE_GRAB_POSE_ONE_START, Preferences.Poses.BLUE_GRAB_POSE_ONE_END))
                .setLinearHeadingInterpolation(Preferences.Poses.BLUE_GRAB_POSE_ONE_START.getHeading(), Preferences.Poses.BLUE_GRAB_POSE_ONE_END.getHeading())
                .build();
        grabOneLaunch = follower.pathBuilder()
                .addPath(new BezierLine(Preferences.Poses.BLUE_GRAB_POSE_ONE_END, Preferences.Poses.BLUE_LAUNCH_POSE))
                .setLinearHeadingInterpolation(Preferences.Poses.BLUE_GRAB_POSE_ONE_END.getHeading(), Preferences.Poses.BLUE_LAUNCH_POSE.getHeading())
                .build();
        grabTwoStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        Preferences.Poses.BLUE_LAUNCH_POSE,
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_START))
                .setLinearHeadingInterpolation(
                        Preferences.Poses.BLUE_LAUNCH_POSE.getHeading(),
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_START.getHeading())
                .build();

        grabTwoEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_START,
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_END))
                .setLinearHeadingInterpolation(
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_START.getHeading(),
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_END.getHeading())
                .build();

        grabTwoLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_END,
                        Preferences.Poses.BLUE_LAUNCH_POSE))
                .setLinearHeadingInterpolation(
                        Preferences.Poses.BLUE_GRAB_POSE_TWO_END.getHeading(),
                        Preferences.Poses.BLUE_LAUNCH_POSE.getHeading())
                .build();
        grabThreeStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        Preferences.Poses.BLUE_LAUNCH_POSE,
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_START))
                .setLinearHeadingInterpolation(
                        Preferences.Poses.BLUE_LAUNCH_POSE.getHeading(),
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_START.getHeading())
                .build();

        grabThreeEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_START,
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_END))
                .setLinearHeadingInterpolation(
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_START.getHeading(),
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_END.getHeading())
                .build();

        grabThreeLaunch = follower.pathBuilder()
                .addPath(new BezierLine(
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_END,
                        Preferences.Poses.BLUE_LAUNCH_POSE))
                .setLinearHeadingInterpolation(
                        Preferences.Poses.BLUE_GRAB_POSE_THREE_END.getHeading(),
                        Preferences.Poses.BLUE_LAUNCH_POSE.getHeading())
                .build();


    }

    @Override
    public void initialize() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap);
        indexerSubsystem = new IndexerSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakerSubsystem = new OuttakerSubsystem(hardwareMap);
        hoodLifterSubsystem = new HoodLifterSubsystem(hardwareMap);

        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Preferences.Poses.BLUE_FAR_START_POSE);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new LaunchCommand(launcherSubsystem),
                new FollowPathCommand(follower, scorePreload),
                new WaitCommand(1000),
                new IntakeCommand(intakeSubsystem),
                new WaitCommand(3000),

                new GoToPoseCommand(follower, Preferences.Poses.BLUE_GATE_RELEASE_POSE),

                new FollowPathCommand(follower, grabOneStart),
                new IntakeCommand(intakeSubsystem),
                new FollowPathCommand(follower, grabOneEnd),
                new StopIntakeCommand(intakeSubsystem),
                new WaitCommand(500),
                new FollowPathCommand(follower, grabOneLaunch),
                new WaitCommand(1000),
                new WaitCommand(3000),

                new FollowPathCommand(follower, grabTwoStart),
                new IntakeCommand(intakeSubsystem),
                new FollowPathCommand(follower, grabTwoEnd),
                new StopIntakeCommand(intakeSubsystem),
                new WaitCommand(500),
                new FollowPathCommand(follower, grabTwoLaunch),
                new WaitCommand(1000),
                new WaitCommand(3000),

                new FollowPathCommand(follower, grabThreeStart),
                new IntakeCommand(intakeSubsystem),
                new FollowPathCommand(follower, grabThreeEnd),
                new StopIntakeCommand(intakeSubsystem),
                new WaitCommand(500),
                new FollowPathCommand(follower, grabThreeLaunch),
                new WaitCommand(1000),
                new WaitCommand(3000),
                new StopLaunchCommand(launcherSubsystem)
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}