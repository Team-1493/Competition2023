package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.FollowLimelight;
import frc.robot.commands.ShootCubeManual;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ShootCubeAuto{
    private SwerveDrive m_SwerveDrive;
    private IntakeSystem m_IntakeSystem;
    private Limelight m_Limelight;

    public SequentialCommandGroup getShootCubeAutoCommand(IntakeSystem intakeSystem, SwerveDrive swerveDrive, Limelight limelight){
        m_IntakeSystem = intakeSystem;
        m_SwerveDrive = swerveDrive;
        m_Limelight = limelight;
        return new SequentialCommandGroup(new FollowLimelight(m_SwerveDrive,m_Limelight, true, false),new ShootCubeManual(m_IntakeSystem, 1));
    }
}