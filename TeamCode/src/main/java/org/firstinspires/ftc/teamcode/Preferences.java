package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Preferences {

    public static final String RIGHT_FRONT_MOTOR = "rightFrontMotor";
    public static final String RIGHT_REAR_MOTOR = "rightBackMotor";
    public static final String LEFT_REAR_MOTOR = "leftBackMotor";
    public static final String LEFT_FRONT_MOTOR = "leftFrontMotor";
    public static final String INTAKE_MOTOR = "intakerMotor";
    public static final String LAUNCHER_MOTOR = "launcherMotor";
    public static final String INDEXER_SERVO = "indexerServo";
    public static final String LEFT_OUTTAKE_SERVO = "leftOuttakeServo";
    public static final String RIGHT_OUTTAKE_SERVO = "rightOuttakeServo";
    public static final String LEFT_HOOD_SERVO = "leftHoodServo";
    public static final String RIGHT_HOOD_SERVO = "rightHoodServo";
    public static final String LEFT_COLOR_SENSOR = "leftColorSensor";
    public static final String RIGHT_COLOR_SENSOR = "rightColorSensor";
    public static final String PINPOINT_ODOMETRY = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu";

    @Configurable
    public static class Poses {
        public static Pose RED_GOAL_POSE = new Pose(136, 142);
        public static Pose BLUE_GOAL_POSE = new Pose(8, 142);

        public static Pose BLUE_FAR_START_POSE = new Pose(56, 8, Math.toRadians(90));
        public static Pose BLUE_LAUNCH_POSE = calculateLaunchPose(60, 12, BLUE_GOAL_POSE);
        public static Pose RED_LAUNCH_POSE = calculateLaunchPose(72, 23, RED_GOAL_POSE);
        public static Pose BLUE_GATE_RELEASE_POSE = new Pose(16, 70, Math.toRadians(180));
        public static Pose RED_GATE_RELEASE_POSE = new Pose(127, 70, Math.toRadians(90));
        public static Pose BLUE_PARK_POSE = new Pose(105.35, 33.35, Math.toRadians(90));
        public static Pose BLUE_GRAB_POSE_ONE_START = new Pose(43, 84, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_ONE_END = new Pose(18, 84, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_TWO_START = new Pose(43, 60, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_TWO_END = new Pose(18, 60, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_THREE_START = new Pose(43, 36, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_THREE_END = new Pose(18, 36, Math.toRadians(180));
        public static Pose CLOSE_BLUE_LAUNCH_POSE = calculateLaunchPose(60, 84, BLUE_GOAL_POSE);

        private static Pose calculateLaunchPose(double x, double y, Pose goalPose) {
            double deltaX = goalPose.getX() - x;
            double deltaY = goalPose.getY() - y;
            double angleToGoal = Math.atan2(deltaY, deltaX);
            double heading = angleToGoal + Math.PI;
            return new Pose(x, y, heading);
        }
    }

    @Configurable
    public static class Intake {
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    }

    @Configurable
    public static class Launcher {
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
    }

    @Configurable
    public static class Hood {
        public static double POSITION_DOWN = 0;
        public static double POSITION_UP = 1.0;
        public static boolean INVERT_SERVO = false;
    }

    @Configurable
    public static class Indexer {
    }

    @Configurable
    public static class Outtake {
        public static boolean INVERT_DIRECTION = false;
    }

    @Configurable
    public static class Limelight {
        public static int PIPELINE_BLUE_GOAL = 0;
        public static int PIPELINE_RED_GOAL = 1;
        public static double TURN_KP = 0.02;
        public static double TURN_KI = 0.0;
        public static double TURN_KD = 0.001;
        public static double TURN_KF = 0.0;
        public static double TURN_MIN_POWER = 0.1;
        public static double TURN_MAX_POWER = 0.5;
        public static double TURN_TOLERANCE = 2.0;
        public static double SEARCH_ROTATION_SPEED = 0.3;
    }

    private Preferences() {
        throw new AssertionError("Don't instantiate this dawg.");
    }
}