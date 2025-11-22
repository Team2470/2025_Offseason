// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

//Source Code imports
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotIdentity;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.shoulder.ShoulderIOTalonFX;
import frc.robot.subsystems.arm.wrist.WristIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.drive.SwerveIOCTRE;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.superstructure.Superstructure;

import java.util.Optional;

// AdvantageKit imports
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// Pheonix6 imports
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

// WPILib imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    // TODO: The HP Intake and Climb subsystems are currently commented out as they have not been fully implemented
    // private final HPIntakeSubsystem hpIntakeSubsystem;
    // private final ClimbSubsystem climbSubsystem;
    private final Superstructure superstructure;

    private final CommandXboxController controller = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Routine");

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotConstants constants = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants =
                constants.getModuleConstants();

        swerveSubsystem = new SwerveSubsystem(
                new SwerveIOCTRE(constants.getSwerveDrivetrainConstants(), constants.getModuleConstants()), controller, moduleConstants[0].SpeedAt12Volts,
                    moduleConstants[0].SpeedAt12Volts / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));

        // hpIntakeSubsystem = new HPIntakeSubsystem(new IntakeIOPhoenix6(constants.getPortConfiguration()));
        var cameraConfigs = constants.getCameraConfigurations();
        var visionIOs = new VisionIOLimelight[cameraConfigs.size()];
        for (int i = 0; i < visionIOs.length; i++) {
            visionIOs[i] = new VisionIOLimelight(cameraConfigs.get(i));
        }
        visionSubsystem = new VisionSubsystem(visionIOs);
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFX(constants.getPortConfiguration(), constants.getArmConfiguration()));

        armSubsystem = new ArmSubsystem(
                new ShoulderIOTalonFX(constants.getPortConfiguration(), constants.getArmConfiguration()),
                new WristIOTalonFX(constants.getPortConfiguration(), constants.getArmConfiguration()));

        // climbSubsystem = new ClimbSubsystem(new ClimberIOPhoenix6(constants.getPortConfiguration()));

        superstructure = new Superstructure(
                swerveSubsystem,
                elevatorSubsystem,
                armSubsystem
                // climbSubsystem,
                // hpIntakeSubsystem
                );

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Left Bumper is normal bumper on top of controller
        controller
            .leftBumper()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.LEFT_L4,
                    Superstructure.WantedSuperState.NET,
                    Superstructure.WantedSuperState.GROUND_ALGAE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // Left Trigger is normal trigger on top of controller
        controller
            .leftTrigger()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.LEFT_L3,
                    Superstructure.WantedSuperState.PROCESSOR,
                    Superstructure.WantedSuperState.REEF_ALGAE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // Y-Button is mapped to top left paddle on back of controller
        controller
            .y()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.LEFT_L2,
                    Superstructure.WantedSuperState.DRIVE,
                    Superstructure.WantedSuperState.REEF_ALGAE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // X-Button is mapped to bottom left paddle on back of controller
        controller
            .x()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.L1,
                    Superstructure.WantedSuperState.DRIVE,
                    Superstructure.WantedSuperState.HP_INAKE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // Right Bumper is normal bumper on top of controller
        controller
            .rightBumper()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.RIGHT_L4,
                    Superstructure.WantedSuperState.NET,
                    Superstructure.WantedSuperState.GROUND_ALGAE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // Right Trigger is normal trigger on top of controller
        controller
            .rightTrigger()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.RIGHT_L3,
                    Superstructure.WantedSuperState.PROCESSOR,
                    Superstructure.WantedSuperState.REEF_ALGAE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // B-Button is mapped to top right paddle on back of controller
        controller
            .b()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.RIGHT_L2,
                    Superstructure.WantedSuperState.DRIVE,
                    Superstructure.WantedSuperState.REEF_ALGAE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // A-Button is mapped to bottom right paddle on back of controller
        controller
            .a()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.L1,
                    Superstructure.WantedSuperState.DRIVE,
                    Superstructure.WantedSuperState.HP_INAKE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // D-Pad Up is used to score game peice
        controller
            .povUp()
            .onTrue(superstructure.configureButtonBindings(
                    Superstructure.WantedSuperState.SCORE,
                    Superstructure.WantedSuperState.SCORE,
                    Superstructure.WantedSuperState.SCORE))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // D-Pad Left is used to climb set up
        controller
            .povLeft()
            .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB_SETUP))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // D-Pad Right is used to climb up
        controller
            .povRight()
            .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB_PULL))
            .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DRIVE));

        // Back Button is used to reset robot heading based on alliance color
        controller
            .back()
            .onTrue(new InstantCommand(swerveSubsystem::resetRotationBasedOnAlliance));

        // Start button homes the superstructure
        controller
            .start()
            .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.HOME));
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      return autoChooser.get();
    }
}
