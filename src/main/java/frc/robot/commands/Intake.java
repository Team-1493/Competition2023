package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class Intake extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 

    private double intakeTargetPos;
    private double intakeThreshold;
    private ArmSubsystem m_ArmSubsystem;
    private IntakeSystem m_IntakeSystem;

  public Intake(ArmSubsystem arm,IntakeSystem intake) {
    m_ArmSubsystem = arm;
    m_IntakeSystem = intake;
    SmartDashboard.putNumber("Intake Arm Target", intakeTargetPos);
    SmartDashboard.putNumber("Intake Start Threshold", intakeThreshold);

    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTargetPos = SmartDashboard.getNumber("Stow Arm Target", intakeTargetPos);
    intakeThreshold = SmartDashboard.getNumber("Intake Start Threshold", intakeThreshold);
    m_ArmSubsystem.setTgtPositionInCounts(intakeTargetPos);

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    if (m_ArmSubsystem.getCounts() > intakeThreshold){
        m_IntakeSystem.IntakeCube();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSystem.HasCube();
  }
}

