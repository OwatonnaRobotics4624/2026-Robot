// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Operator
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //Autos
  public static class AutosConstants {
  
  }

  //Vision
  public static class VisionConstants {
    /** Change these to your limelight NT table names. */
    public static final String kLLFront = "limelight-front";
    public static final String kLLLeft  = "limelight-left";
    public static final String kLLRight = "limelight-right";

    /** If robot is spinning faster than this, ignore vision updates (deg/sec). */
    public static final double kMaxYawRateDegPerSec = 720.0; // common rule of thumb :contentReference[oaicite:3]{index=3}

    /** Heading std dev (rad). Keep large to mostly trust gyro for heading. :contentReference[oaicite:4]{index=4} */
    public static final double kThetaStdRad = 999.0;

    /** Base XY std dev (meters) when tags are good. Tune. */
    public static final double kBaseXYStd = 0.15;

    /** How much to trust measurements as tag distance grows (bigger = trust less). Tune. */
    public static final double kDistancePenalty = 0.08;

    /** Extra trust bonus for seeing multiple tags (bigger = trust more). Tune. */
    public static final double kMultiTagBonus = 0.7;
  }
  
  //Intake
  public static class IntakeConstants {
    // Hardware
    public static final int kIntakeMotorCAN = 25; // <-- change
    public static final boolean kMotorInverted = false;

    // Intake tube geometry
    public static final double kIntakeTubeDiameterMeters = 0.022225; //7/8 of an inch
    public static final double kIntakeTubeRadiusMeters = kIntakeTubeDiameterMeters / 2.0;

    // Gear ratio: motorRotations * kGearRatio = IntakeTubeRotations
    public static final double kGearRatio = 1.0;

    // Limits
    public static final double kMinRPM = 0;
    public static final double kMaxRPM = 3000; // tune safe max

    // Closed-loop (starter values; tune)
    public static final double kP = 0.0002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kFF = 0.00017;

    // Ready tolerance
    public static final double kRPMTolerance = 75.0;

    //Rpms
    public static final double kFastIntakeRpm = 10000;
    public static final double kSafeIntakeRpm = 7000;
    public static final double kSlowIntakeRpm = 4000;
    

  }

  //Intake Pivot
  public static class IntakePivotConstants {
     // Hardware
    public static final int kIntakePivotMotorCAN = 26; // <-- change
    public static final boolean kMotorInverted = false;

    // Intake Pivot geometry
    

    // Gear ratio: motorRotations * kGearRatio = IntakePivotRotations
    public static final double kGearRatio = 1.0;

    // Angle Limits
    public static final Angle kMinAngle = Angle.ofBaseUnits(-10, Degrees); //-10 degrees
    public static final Angle kMaxAngle = Angle.ofBaseUnits(100, Degrees); //100 degrees

    // Angles
    public static final Angle kUpAngle = Angle.ofBaseUnits(0, Degrees); //-10 degrees
    public static final Angle kDownAngle = Angle.ofBaseUnits(90, Degrees); //-10 degrees
    
    // Closed-loop (starter values; tune)
    public static final double kP = 0.0002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kFF = 0.00017;

    //Angle tolerance
    public static final Angle kAngleTolerance = Angle.ofBaseUnits(5, Degrees);
  }
  //Shooter  
  public static class ShooterConstants {
    // Hardware
    public static final int kShooterMotorCAN = 20; // <-- change
    public static final boolean kMotorInverted = false;

    // Flywheel geometry
    public static final double kFlywheelDiameterMeters = 0.1016; // 4 in
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;

    // Gear ratio: motorRotations * kGearRatio = flywheelRotations
    public static final double kGearRatio = 1.0;

    // Exit speed ~= wheel surface speed * this
    public static final double kExitSpeedPerWheelSpeed = 0.85; // tune

    // Limits
    public static final double kMinRPM = 0;
    public static final double kMaxRPM = 6000; // tune safe max

    // Closed-loop (starter values; tune)
    public static final double kP = 0.0002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double feedForwardkV = 0.00017;

    // Ready tolerance
    public static final double kRPMTolerance = 75.0;
  }
  
  //Hood
  public static class HoodConstants {
    public static final int kHoodCAN = 30; // <-- change

    // ✅ Phoenix 6 wants an enum, not boolean                     // bruh not the emojis...
    // Pick whichever makes "positive setpoint" move hood the direction you want.
    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    // If it goes the wrong way, swap to:
    // public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

    // Hood angle limits (deg)
    public static final double kMinAngleDeg = 50.0;
    public static final double kMaxAngleDeg = 60.0;

    // Conversion: hood degrees -> mechanism rotations
    public static final double kMechanismRotPerDeg = 1.0 / 360.0; // placeholder (fix later)

    // Sensor rotations / mechanism rotations (gearbox between motor and hood)
    public static final double kSensorToMechanismRatio = 1.0; // set if geared

    // Motion Magic constraints (mechanism rotations/sec and /sec^2)
    public static final double kCruiseVelocityRps = 2.0;
    public static final double kAccelerationRps2 = 6.0;

    // PID (starter)
    public static final double kP = 40.0;
    public static final double kI = 0.0;
    public static final double kD = 0.2;

    public static final double kAngleToleranceDeg = 1.0;
  }
  
  //Aim Hood and shooter
  public static class AimHoodAndShooterConstants {
    public static final Translation2d kBlueScoreXY = new Translation2d(4.637, 4.072);
    public static final Translation2d kRedScoreXY  = new Translation2d(11.984, 4.072);

    public static final double kTargetHeightMeters = 1.8288; // 6 ft
    public static final double kShooterExitHeightMeters = 0.90;

    public static final double kMaxApexHeightMeters = 3.048; // 10 ft
    public static final double kAngleStepDeg = 0.5;

    public static final double kMinExitSpeedMps = 3.0;
    public static final double kMaxExitSpeedMps = 25.0;

    public static final Translation2d kFallbackScoreXY = kBlueScoreXY;
  }

  //Shooter Speed From Position
  public static class ShooterSpeedFromPositionConstants {
    public static final Translation2d kBlueScoreXY = new Translation2d(4.637, 4.072);
    public static final Translation2d kRedScoreXY  = new Translation2d(11.984, 4.072);

    public static final double kTargetHeightMeters = 1.8288; // 6 ft
    public static final double kShooterExitHeightMeters = 0.90; // measure this

    public static final double kMinExitSpeedMps = 3.0;
    public static final double kMaxExitSpeedMps = 25.0;

    public static final Translation2d kFallbackScoreXY = kBlueScoreXY;
  }


}

