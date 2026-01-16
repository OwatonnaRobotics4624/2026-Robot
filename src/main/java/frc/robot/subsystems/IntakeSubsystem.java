package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {
    

    
    
    private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.kIntakeMotorCAN, MotorType.kBrushless);

    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    private final SparkClosedLoopController pid = intakeMotor.getClosedLoopController();
    
    
    private double targetIntakeRPM = 0.0;

    /** This is the subsystem that controls the sillicone tube on the intake */
    public IntakeSubsystem() {
        
         
        // REVLib 2025+ uses config objects + configure()
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(IntakeConstants.kMotorInverted);

        // Closed-loop gains live in the config now
        config.closedLoop
            .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
            .outputRange(-1.0, 1.0);

        // If this line errors on your exact REVLib version, tell me and I'll switch to:
        // config.closedLoop.feedForward.<something...>
        //config.closedLoop.velocityFF(IntakeConstants.kFF);
        config.closedLoop.feedForward.kV(IntakeConstants.kFF);

        // Apply config:
        // - Reset safe params so you're in a known state
        // - Persist so it survives brownouts/power cycles
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    /** Sets intake RPM (NOT motor RPM). */
    public void setIntakeRPM(double intakeRPM) {
        intakeRPM = MathUtil.clamp(intakeRPM, IntakeConstants.kMinRPM, IntakeConstants.kMaxRPM);
        targetIntakeRPM = intakeRPM;

        // Convert flywheel RPM -> motor RPM based on gear ratio
        double motorRPM = intakeRPM / IntakeConstants.kGearRatio;

        
        
        // REVLib 2025+: setSetpoint (setReference is deprecated)
        pid.setSetpoint(motorRPM, SparkBase.ControlType.kVelocity);
        
    }

    public void stop() {
        targetIntakeRPM = 0.0;
        intakeMotor.stopMotor();
        
    }
    
    /** Returns intake RPM estimate from motor encoder RPM and gear ratio. */
    public double getIntakeRPM() {
        double motorRPM = encoder.getVelocity(); // RPM
        return motorRPM * IntakeConstants.kGearRatio;
    }

    public double getTargetIntakeRPM() {
        return targetIntakeRPM;
    }

    public boolean atSetpoint() {
        return Math.abs(getIntakeRPM() - targetIntakeRPM) <= IntakeConstants.kRPMTolerance;
    }



}
