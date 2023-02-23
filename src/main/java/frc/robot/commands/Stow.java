package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class Stow extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 

    private double stowTargetPos = 1241;
    private double stowThreshold;
    private double stowFinishPos; // when this is reached it's safe to assume we are stowed
    private ArmSubsystem m_ArmSubsystem;
    private IntakeSystem m_IntakeSystem;

  public Stow(ArmSubsystem arm,IntakeSystem intake) {
    m_ArmSubsystem = arm;
    m_IntakeSystem = intake;
    SmartDashboard.putNumber("Stow Arm Target", SmartDashboard.getNumber("Stow Arm Target", stowTargetPos));
    SmartDashboard.putNumber("Stow Stage2 Threshold", SmartDashboard.getNumber("Stow Stage2 Threshold", stowThreshold));
    SmartDashboard.putNumber("Stow Finish Position", SmartDashboard.getNumber("Stow Finish Position", stowFinishPos));

    addRequirements(arm,intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stowTargetPos = SmartDashboard.getNumber("Stow Arm Target", stowTargetPos);
    stowThreshold = SmartDashboard.getNumber("Stow Stage2 Threshold", stowThreshold);
    stowFinishPos = SmartDashboard.getNumber("Stow Finish Position", stowFinishPos);
    m_ArmSubsystem.setTgtPositionInCounts(1500);
    m_IntakeSystem.runFrontIntakeBack();

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {

    // if (m_ArmSubsystem.getCounts() < stowThreshold){
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.StopMotors();
    m_IntakeSystem.StopMotors().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_ArmSubsystem.getCounts());
    return m_ArmSubsystem.getCounts() <= stowFinishPos;
  }
}

