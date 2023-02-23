package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class RickCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Orchestra orch= new Orchestra();
  public RickCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    orch.loadMusic("test.chrp");
    orch.addInstrument(new TalonFX(11));
    orch.addInstrument(new TalonFX(10));
    orch.play();
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
    orch.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

