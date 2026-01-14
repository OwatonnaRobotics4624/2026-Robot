package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class IntakeSubsystem {
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

    //if we are using a neo or vortex (spark flex)
    /*
    private final SparkFlex intakeMotor =
      new SparkFlex(IntakeConstants.kShooterMotorCAN, MotorType.kBrushless);

    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    private final SparkClosedLoopController pid = intakeMotor.getClosedLoopController();
    */
    
    private double targetIntakeRPM = 0.0;


    public IntakeSubsystem() {
        //if we are using a neo or vortex (spark flex)
        /* 
        // REVLib 2025+ uses config objects + configure()
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(IntakeConstants.kMotorInverted);

        // Closed-loop gains live in the config now
        config.closedLoop
            .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
            .outputRange(-1.0, 1.0);

        // If this line errors on your exact REVLib version, tell me and I'll switch to:
        // config.closedLoop.feedForward.<something...>
        //config.closedLoop.velocityFF(IntakeConstants.kFF);
        config.closedLoop.feedForward.kV(IntakeConstants.feedForwardkV);

        // Apply config:
        // - Reset safe params so you're in a known state
        // - Persist so it survives brownouts/power cycles
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        */
    }

    /** Sets intake RPM (NOT motor RPM). */
    public void setIntakeRPM(double intakeRPM) {
        intakeRPM = MathUtil.clamp(intakeRPM, IntakeConstants.kMinRPM, IntakeConstants.kMaxRPM);
        targetIntakeRPM = intakeRPM;

        // Convert flywheel RPM -> motor RPM based on gear ratio
        double motorRPM = intakeRPM / ShooterConstants.kGearRatio;

        //if we are using a neo or vortex (spark flex)
        /* 
        // REVLib 2025+: setSetpoint (setReference is deprecated)
        pid.setSetpoint(motorRPM, SparkBase.ControlType.kVelocity);
        */
    }

    public void stop() {
        targetIntakeRPM = 0.0;

        //if we are using a neo or vortex (spark flex)
        /*
        intakeMotor.stopMotor();
        */
    }
    
    /** Returns intake RPM estimate from motor encoder RPM and gear ratio. */
    public double getIntakeRPM() {
        //if we are using a neo or vortex (spark flex)
        /*
        double motorRPM = encoder.getVelocity(); // RPM
        return motorRPM * ShooterConstants.kGearRatio;
        */
        return 0;
    }

    public double getTargetIntakeRPM() {
        return targetIntakeRPM;
    }

    public boolean atSetpoint() {
        return Math.abs(getIntakeRPM() - targetIntakeRPM) <= IntakeConstants.kRPMTolerance;
    }

    

}
