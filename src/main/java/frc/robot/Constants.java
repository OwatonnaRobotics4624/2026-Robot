// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

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
  //Intake
  public static class IntakeConstants {
    // Hardware
    public static final int kShooterMotorCAN = 21; // <-- change
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

}
