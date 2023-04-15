package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.15;
    //public static final SlewRateLimiter rateLimit = new SlewRateLimiter(1);

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false;
        public static final int maxVoltage = 12;
        
        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.5);
        public static final double wheelBase = Units.inchesToMeters(19.5);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.18249 / 12); 
        public static final double driveKV = (2.3025 / 12);
        public static final double driveKA = (0.38023 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 6380.0 / 60.0 * ((14.0/50.0) *(27.0 / 17.0) * ( 15.0/45.0)) * Units.inchesToMeters(4) * Math.PI;
        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed/Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

        /* Neutral Modes */
        public static NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(113.82); //113.82
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8.53); //8.53
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(145.20); //145.2
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(130.34); //130.34
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static enum JointMovementType {
        MOVE_TOGETHER,
        WRIST_FIRST,
        ARM_FIRST
    }
    public static final class CollectionConstants {
        public static final int collectionBeamBreak = 1;
        public static final int collectionMotor = 12;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_MASTER = 9;
        public static final int ARM_MOTOR_SLAVE = 10;

        // Setpoints
        public static final int HOME = 0;
        public static final int ACQUIRE_FROM_CUBE_FLOOR = 6000;
        public static final int ACQUIRE_FROM_CONE_FLOOR = 30000;
        public static final int ACQUIRE_FROM_DOS = 34000;
        public static final int ACQUIRE_FROM_SIS = 25000;

        public static final int SCORE_IN_HIGH_CONE = 60000;
        public static final int SCORE_IN_HIGH_CUBE = 47000; // 14200
        public static final int SCORE_IN_MID_CONE = 52000;
        public static final int SCORE_IN_MID_CUBE = 24000;

        // Movement
        public static final float UP_kP = .06030624264f;
        public static final float UP_kI = 0f;
        public static final float UP_kD = 0f;
        public static final float UP_kF = 0.1f;

        public static final float DOWN_kP = .1265760198f;
        public static final float DOWN_kI = 0f;
        public static final float DOWN_kD = 0f;
        public static final float DOWN_kF = 0.1f;
        
        public static int PEAK_VELOCITY_UP = 13360;
        public static final float PERCENT_OF_PEAK_UP = .85f;
        public static final float CRUISE_VELOCITY_ACCEL_UP = PEAK_VELOCITY_UP * PERCENT_OF_PEAK_UP;

        public static int PEAK_VELOCITY_DOWN = 8090;
        public static final float PERCENT_OF_PEAK_DOWN = .85f;
        public static final float CRUISE_VELOCITY_ACCEL_DOWN = PEAK_VELOCITY_DOWN * PERCENT_OF_PEAK_DOWN;
    }

    public static final class WristConstants {
        public static final int WRIST_MOTOR = 11;

        // Setpoints
        public static final int HOME = 0;
        public static final int ACQUIRE_FROM_CUBE_FLOOR = 47000 - 9000;
        public static final int ACQUIRE_FROM_CONE_FLOOR = 111000 - 9000;
        public static final int ACQUIRE_FROM_DOS = 58000 - 20000;
        public static final int ACUQIRE_FROM_SIS = 133000 - 9000;

        public static final int SCORE_IN_HIGH_CONE = 102000 - 9000; 
        public static final int SCORE_IN_HIGH_CUBE = 46000 - 9000; // 16500 shoot
        public static final int SCORE_IN_MID_CONE = 98000 - 9000;
        public static final int SCORE_IN_MID_CUBE = 25000;
        

        // Movement
        public static final float UP_kP = .0367156687f;
        public static final float UP_kI = 0f;
        public static final float UP_kD = 0f;
        public static final float UP_kF = 0.1f;

        public static final float DOWN_kP = .0367156687f;
        public static final float DOWN_kI = 0f;
        public static final float DOWN_kD = 0f;
        public static final float DOWN_kF = 0.1f;

        public static int PEAK_VELOCITY_UP = 14940;
        public static final float PERCENT_OF_PEAK_UP = 1f;
        public static final float CRUISE_VELOCITY_ACCEL_UP = PEAK_VELOCITY_UP * PERCENT_OF_PEAK_UP;

        public static int PEAK_VELOCITY_DOWN = 14940;
        public static final float PERCENT_OF_PEAK_DOWN = 1f;
        public static final float CRUISE_VELOCITY_ACCEL_DOWN = PEAK_VELOCITY_DOWN * PERCENT_OF_PEAK_DOWN;

    }

    public static final class SuperstructureConstants {
        public static enum ROBOT_STATE {
            ACQUIRE_CONE_DOS,
            ACQUIRE_CUBE_DOS,
            ACQUIRE_CONE_FLOOR,
            ACQUIRE_CUBE_FLOOR
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = /*Swerve.maxSpeed * .75*/4;
        public static final double kMaxAccelerationMetersPerSecondSquared = /*kMaxSpeedMetersPerSecond * 1.5 */ 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);
    
        public static final double kPXController = 5; // Different max speeds require different PIDs.
        public static final double kDXController = 0;
        public static final double kPYController = kPXController;
        public static final double kDYController = kDXController;
        public static final double kPThetaController = 2;
        public static final double kDThetaController = 0;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
