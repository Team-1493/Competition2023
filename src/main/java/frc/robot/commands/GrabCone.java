package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class GrabCone extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private IntakeSystem m_IntakeSystem;
    private ArmSubsystem m_arm;

  public GrabCone(IntakeSystem intake, ArmSubsystem arm) {
    m_IntakeSystem = intake;
    m_arm=arm;
    addRequirements(intake,arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.GrabCone();
    m_arm.motorActive=true;

  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    m_arm.setPositionInCounts(m_arm.posConeGrab);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

