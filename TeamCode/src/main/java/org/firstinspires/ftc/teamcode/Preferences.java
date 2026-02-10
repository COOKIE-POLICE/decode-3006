package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// http://192.168.43.1:8001/
public class Preferences {

    public static final String RIGHT_FRONT_MOTOR = "rightFrontMotor";
    public static final String RIGHT_REAR_MOTOR = "rightBackMotor";
    public static final String LEFT_REAR_MOTOR = "leftBackMotor";
    public static final String LEFT_FRONT_MOTOR = "leftFrontMotor";
    public static final String INTAKE_MOTOR = "intakerMotor";
    public static final String BLOCKER_SERVO = "blockerServo";
    public static final String LAUNCHER_MOTOR = "launcherMotor";
    public static final String PINPOINT_ODOMETRY = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu";

    @Configurable
    public static class Poses {

        public static class StartingPoses {
            public static Pose BLUE_CLOSE_START_POSE = new Pose(22, 125, Math.toRadians(324));
            public static Pose BLUE_FAR_START_POSE = new Pose(56, 8, Math.toRadians(90));
            public static Pose RED_FAR_START_POSE = BLUE_FAR_START_POSE.mirror();
            public static Pose RED_CLOSE_START_POSE = BLUE_CLOSE_START_POSE.mirror();
        }
        public static class CloseLaunchingPoses {
            public static Pose CLOSE_BLUE_LAUNCH_POSE = calculateLaunchPose(60, 84, BLUE_GOAL_POSE);
            public static Pose CLOSE_RED_LAUNCH_POSE = CLOSE_BLUE_LAUNCH_POSE.mirror();
        }
        public static class LaunchingPoses {
            public static Pose BLUE_LAUNCH_POSE = calculateLaunchPose(60, 14, BLUE_GOAL_POSE);
            public static Pose RED_LAUNCH_POSE = BLUE_LAUNCH_POSE.mirror();
        }
        public static class MovementPoses {
            public static Pose MOVEMENT_BLUE_FAR_POSE = new Pose(42, 36, Math.toRadians(180));
            public static Pose MOVEMENT_RED_FAR_POSE = MOVEMENT_BLUE_FAR_POSE.mirror();
            public static Pose MOVEMENT_BLUE_CLOSE_POSE = new Pose();
            public static Pose MOVEMENT_RED_CLOSE_POSE = new Pose();
        }

        public static Pose BLUE_GOAL_POSE = new Pose(13, 137);
        public static Pose RED_GOAL_POSE = BLUE_GOAL_POSE.mirror();
        public static Pose BLUE_GATE_RELEASE_POSE = new Pose(16, 70, Math.toRadians(180));
        public static Pose RED_GATE_RELEASE_POSE = new Pose(128, 70, Math.toRadians(0));
        public static Pose BLUE_PARK_POSE = new Pose(105.35, 33.35, Math.toRadians(90));
        public static Pose BLUE_GRAB_POSE_ONE_START = new Pose(43, 84, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_ONE_END = new Pose(18, 84, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_TWO_START = new Pose(43, 60, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_TWO_END = new Pose(18, 60, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_THREE_START = new Pose(43, 36, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_THREE_END = new Pose(18, 36, Math.toRadians(180));
        public static Pose RED_PARK_POSE = new Pose(38.5, 33.5, Math.toRadians(0));

        public static Pose BLUE_HUMAN_PLAYER_GRAB_POSE_START = new Pose(13, 22, Math.toRadians(240));
        public static Pose BLUE_HUMAN_PLAYER_GRAB_POSE_END = new Pose(9.75, 9.55, Math.toRadians(270));

        public static Pose RED_HUMAN_PLAYER_GRAB_POSE_START = BLUE_HUMAN_PLAYER_GRAB_POSE_START.mirror();
        public static Pose RED_HUMAN_PLAYER_GRAB_POSE_END = BLUE_HUMAN_PLAYER_GRAB_POSE_END.mirror();

        private static Pose calculateLaunchPose(double x, double y, Pose goalPose) {
            double deltaX = goalPose.getX() - x;
            double deltaY = goalPose.getY() - y;
            double angleToGoal = Math.atan2(deltaY, deltaX);
            double heading = angleToGoal;
            return new Pose(x, y, heading);
        }
    }

    @Configurable
    public static class Intake {
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    }

    @Configurable
    public static class Launcher {
        public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5.0, 0.0, 0.0, 12.0);
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static double TARGET_VELOCITY = 50;
        public static double P_UP = 75.0;
        public static double P_DOWN = 0.1;
    }
    @Configurable
    public static class BlockerServo {
        public static double BLOCK_POSITION = 1.0;
        public static double ADMIT_POSITION = 0.658;
    }

    @Configurable
    public static class Limelight {
        public static int PIPELINE_BLUE_GOAL = 0;
        public static int PIPELINE_RED_GOAL = 1;
    }

    private Preferences() {
        throw new AssertionError("Don't instantiate this dawg.");
    }
}