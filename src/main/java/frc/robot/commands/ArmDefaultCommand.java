package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSystem;

public class ArmDefaultCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 
    private ArmSubsystem arm;
    private final Supplier<double[]> stickState;
    private double setPoint;
    private boolean motorActive;

  public ArmDefaultCommand(ArmSubsystem m_arm, Supplier<double[]> m_stickState) {
    arm= m_arm;
    stickState=m_stickState;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is schedule d.
  @Override
  public void execute() {
    double stickValue=stickState.get()[2];
    if (Math.abs(stickValue)>0.02)  {
//        arm.setArmWithStick(stickValue);
//        arm.setPoint=arm.getCounts();
    }
    else {
      arm.setPositionInCounts(arm.setPoint);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

