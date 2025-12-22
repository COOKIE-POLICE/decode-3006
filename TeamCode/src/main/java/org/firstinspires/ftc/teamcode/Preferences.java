package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// 192.168.43.1:8001
public class Preferences {

    public static final String RIGHT_FRONT_MOTOR = "rightFrontMotor"; // Expansion 0
    public static final String RIGHT_REAR_MOTOR = "rightBackMotor"; // Expansion 1
    public static final String LEFT_REAR_MOTOR = "leftBackMotor"; // Expansion 2
    public static final String LEFT_FRONT_MOTOR = "leftFrontMotor"; // Expansion 3
    public static final String INTAKE_MOTOR = "intakerMotor"; // Control 0
    public static final String LAUNCHER_MOTOR = "launcherMotor"; // Control 1
    public static final String INDEXER_SERVO = "indexerServo"; // Servo Hub 5th Port
    public static final String LEFT_OUTTAKE_SERVO = "leftOuttakeServo"; // Servo Hub 2nd Port
    public static final String RIGHT_OUTTAKE_SERVO = "rightOuttakeServo"; // Servo Hub 0th Port
    public static final String LEFT_HOOD_SERVO = "leftHoodServo"; // Servo Hub 1st Port
    public static final String RIGHT_HOOD_SERVO = "rightHoodServo"; // Servo Hub 3rd Port
    public static final String LEFT_COLOR_SENSOR = "leftColorSensor"; // Expansion, port 0
    public static final String RIGHT_COLOR_SENSOR = "rightColorSensor"; // Expansion, port 1
    public static final String PINPOINT_ODOMETRY = "pinpoint"; // Control Hub I2C Port 2
    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu"; // Control Hub I2C Port 0

    @Configurable
    public static class Poses {
        public static Pose BLUE_FAR_START_POSE = new Pose(56, 8, Math.toRadians(90));
        public static Pose BLUE_LAUNCH_POSE = new Pose(60, 91, Math.toRadians(322));
        public static Pose RED_LAUNCH_POSE = new Pose(72, 23, Math.toRadians(63));
        public static Pose BLUE_GATE_RELEASE_POSE = new Pose(16, 70, Math.toRadians(180));
        public static Pose RED_GATE_RELEASE_POSE = new Pose(127, 70, Math.toRadians(90));
        public static Pose BLUE_PARK_POSE = new Pose(105.35, 33.35, Math.toRadians(90));
        public static Pose BLUE_GRAB_POSE_ONE_START = new Pose(43, 84, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_ONE_END = new Pose(18, 84, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_TWO_START = new Pose(43, 60, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_TWO_END = new Pose(18, 60, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_THREE_START = new Pose(43, 36, Math.toRadians(180));
        public static Pose BLUE_GRAB_POSE_THREE_END = new Pose(18, 36, Math.toRadians(180));
        public static Pose RED_GOAL_POSE = new Pose(136, 142);
        public static Pose BLUE_GOAL_POSE = new Pose(8, 142);



    }
    
    @Configurable
    public static class Intake {
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    }
    @Configurable
    public static class Launcher {
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static double LAUNCH_VELOCITY = 1700.0;
        public static PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(0.05, 0, 0.00015, 14.5);
    }
    
    @Configurable
    public static class Hood {
        public static double POSITION_DOWN = 1.0;
        public static double POSITION_UP = 0.475;
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
