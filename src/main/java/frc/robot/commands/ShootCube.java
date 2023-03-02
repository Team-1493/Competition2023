package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class ShootCube extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private IntakeSystem m_IntakeSystem;
    private boolean shoot;
    private int speedLevel;  
  public ShootCube(IntakeSystem intake, int m_speedLevel) {
    m_IntakeSystem = intake;
    speedLevel=m_speedLevel;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSystem.botConveyorShootCube(speedLevel);
    shoot=false;
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    if(m_IntakeSystem.botConveyorAtShootSpeed()) shoot=true;
    if(shoot) m_IntakeSystem.topConveyorShootCube();
    SmartDashboard.putNumber("TopConveyorSpeed_RPM", m_IntakeSystem.getTopConveyorSpeed()*600/2048);
    SmartDashboard.putNumber("BotConveyorSpeed_RPM", m_IntakeSystem.getBottomConveyorSpeed()*600/2048);
    SmartDashboard.putNumber("BotConveyor CLE", m_IntakeSystem.getBottomConveyorCLE());
    

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

