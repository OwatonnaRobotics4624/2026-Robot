package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;

public class Pickup extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private final IntakePivotSubsystem m_IntakePivotSubsystem;

  /**
  * @param intakeSubsystem Intake Subsystem (sillicone roller motor)
  * @param pivotSubsystem  Pivot Subsystem (Motor that controls the angle of the
  *                        intake)
  */
  public Pickup(IntakeSubsystem intakeSubsystem, IntakePivotSubsystem pivotSubsystem) {
  	this.m_IntakeSubsystem = intakeSubsystem;
    this.m_IntakePivotSubsystem = pivotSubsystem;
    addRequirements(intakeSubsystem, pivotSubsystem);
  }

	@Override
  public void execute() {
    m_IntakeSubsystem.setIntakeRPM(IntakeConstants.kSafeIntakeRpm);
  }


}
