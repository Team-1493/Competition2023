// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutoGenerator;
import frc.robot.commands.DriveStick;
import frc.robot.commands.FollowLimelight;
import frc.robot.commands.ArmStickDrive;
import frc.robot.commands.CubeIntake;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Stow;
import frc.robot.commands.ReflectiveTape;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.Stick;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  private final ArmSubsystem m_ArmSystem = new ArmSubsystem();
  public final SwerveDrive m_swervedriveSystem = new SwerveDrive();
  public final Limelight m_Limelight = new Limelight();
  public final IntakeSystem m_IntakeSystem = new IntakeSystem();
  public final AutoGenerator autoGenerator = new AutoGenerator(m_swervedriveSystem, m_ArmSystem, m_IntakeSystem);
  public final Stick driverJoystick =new Stick(0);
  public final Stick operatorJoystick =new Stick(1);

  public final ReflectiveTape reflectivetape = new ReflectiveTape(m_swervedriveSystem);
  public final Stow stowCommand = new Stow(m_ArmSystem,m_IntakeSystem);
  public final CubeIntake cubeIntake = new CubeIntake(m_ArmSystem,m_IntakeSystem);

  Supplier<double[]> driverStickState = () -> driverJoystick.readStick();
  Supplier<double[]> operatorStickState = () -> operatorJoystick.readStick();


  public final DriveStick driveCommand = new DriveStick(m_swervedriveSystem,driverStickState); 
  public final ArmStickDrive armStickDrive = new ArmStickDrive(m_ArmSystem,operatorStickState);

  public JoystickButton btnAimAtTape = driverJoystick.getButton(1);
  public JoystickButton btnResetGyro = driverJoystick.getButton(2);
  public JoystickButton btnUpdateConstants = driverJoystick.getButton(3);
  public JoystickButton btnFollowLimelight = driverJoystick.getButton(4);
  public JoystickButton coneGrabberForward = driverJoystick.getButton(6); // R1
  public JoystickButton coneGrabberBackward = driverJoystick.getButton(5); //L1
  public JoystickButton btnIntakeCube = driverJoystick.getButton(7);
  public JoystickButton btnGrabCone = driverJoystick.getButton(8);
  public JoystickButton btnStow = driverJoystick.getButton(9);

  public JoystickButton btnArmManual = operatorJoystick.getButton(6);
  public JoystickButton btnOperatorUpdateConstants = operatorJoystick.getButton(3);


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swervedriveSystem.setDefaultCommand(driveCommand);
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

    new Trigger(btnResetGyro).onTrue(new ResetGyro(m_swervedriveSystem));
    new Trigger(btnUpdateConstants).onTrue( new InstantCommand(()-> updateConstants()));    
    new Trigger(btnFollowLimelight).whileTrue(new FollowLimelight(m_swervedriveSystem, m_Limelight));
    new Trigger(btnAimAtTape).whileTrue(reflectivetape);
    new Trigger(btnGrabCone).onTrue( new InstantCommand( ()-> m_IntakeSystem.GrabCone()));
    new Trigger(btnGrabCone).onFalse( new InstantCommand( ()->m_IntakeSystem.StopMotors()));

    new Trigger(btnIntakeCube).whileTrue(cubeIntake);
    new Trigger(btnStow).onTrue(stowCommand);

    new Trigger(driverJoystick.pov0).onTrue(m_swervedriveSystem.rotateInPlace(0.));
    new Trigger(driverJoystick.pov0).onTrue(m_swervedriveSystem.rotateInPlace(0.));
    new Trigger(driverJoystick.pov90).onTrue(m_swervedriveSystem.rotateInPlace(90));
    new Trigger(driverJoystick.pov180).onTrue(m_swervedriveSystem.rotateInPlace(180));   
    new Trigger(driverJoystick.pov270).onTrue(m_swervedriveSystem.rotateInPlace(-90));

    //operator buttons
    new Trigger(btnArmManual).whileTrue(armStickDrive);
    new Trigger(btnOperatorUpdateConstants).onTrue( new InstantCommand(()-> updateConstants()));    
/*  Tayab - you need to rethink these
 
    new Trigger(moveMotorForward).onTrue(grabbingCone.rotateMotorFoward.until(coneLimitSwitch::get));
    new Trigger(moveMotorBackward).onTrue(grabbingCone.rotateMotorBackward.until(coneLimitSwitch::get));

    new Trigger(coneGrabberForwardButton).onTrue(grabbingCone.rotateMotorFoward);
    new Trigger(coneGrabberBackwardButton).onTrue(grabbingCone.rotateMotorBackward);
 */

  }


  public Command getAutonomousCommand1() {
    // An example command will be run in autonomous
    autoGenerator.updatePID();;
    return autoGenerator.autoCommand1();
  }

  public Command getAutonomousCommand2() {
    // An example command will be run in autonomous
    return autoGenerator.autoCommand2();
  }

  public Command getAutonomousCommand3() {
    //An example command will be run in autonomous
    return autoGenerator.autoCommand3();
  }

// We have different PID constants for the drive wheels between teleop and auto
// Switch between slot 0 for teleop and slot 1 for auto 
  public void setPIDslot(int slot){
    m_swervedriveSystem.setPIDSlot(slot);
  }

  public void updateConstants(){
    m_swervedriveSystem.updateConstants();
    m_ArmSystem.updateConstants();
    m_IntakeSystem.UpdateConstants(); 
  }


}
