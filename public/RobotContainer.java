// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.AlectronaLib.SwerveDriveInput;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveStates;
import frc.robot.subsystems.SuperStructure.HopperStates;
import frc.robot.subsystems.SuperStructure.IntakeRollerState;
import frc.robot.subsystems.SuperStructure.IntakeStates;
import frc.robot.subsystems.SuperStructure.ShooterStates;
import frc.robot.subsystems.HoodShooter.ShooterSubsystem;
import frc.robot.subsystems.HoodShooter.ShotCalculator;

public class RobotContainer {
    private boolean isAimingMode;
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final LEDs leds = new LEDs();    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public boolean isLock = false;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ShotCalculator m_ShotCalculator = new ShotCalculator(drivetrain);


    private final SuperStructure m_SuperStructure = new SuperStructure(m_IntakeSubsystem, m_ShooterSubsystem, drivetrain, m_HopperSubsystem);

    private SwerveDriveInput m_DriveInput = new SwerveDriveInput();
    private SwerveDriveInput m_RotInput = new SwerveDriveInput();


    private Translation2d modifiedDriveInput = new Translation2d(0,0);
    private Translation2d modifiedRotInput = new Translation2d(0,0);
    public static final double K_FIELD_LENGTH = 16.54; 
    public static final double K_FIELD_WIDTH = 8.07;   
    private boolean isIdleActive = false;


    public RobotContainer() {
        DataLogManager.logNetworkTables(true);
        // m_SuperStructure.intakeRequest(IntakeStates.HOME); //Uncomment WHEN everything is configured
        createAutos();
        configureBindings();
        isAimingMode = false;
        RobotController.setBrownoutVoltage(6.0);
    }

    public void updateDriveInput(){
        modifiedDriveInput = m_DriveInput.getShapedInput(()-> joystick.getLeftX(),()-> joystick.getLeftY());
        // modifiedDriveInput = new Translation2d((joystick.getLeftX()),(joystick.getLeftY()));

        modifiedRotInput = m_RotInput.getShapedInput(()-> joystick.getRightX(), ()->joystick.getRightY());
        // modifiedRotInput = new Translation2d(joystick.getRightX(), joystick.getRightY());

    }

    
    private void configureBindings(){
        leds.setDefaultCommand(Commands.run(this::updateLEDState, leds).ignoringDisable(true));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.SwerveControllerDrive(
                null,
                () -> modifiedDriveInput.getX(),
                () -> modifiedDriveInput.getY(),
                () -> {
                    if (Math.abs(joystick.getRightX()) > driveConstants.deadband) {
                        return null;
                    } else {
                        return drivetrain.getLastHeldRotation();
                    }
                },
                () -> {
                    if (Math.abs(joystick.getRightX()) > driveConstants.deadband) {
                        return modifiedRotInput.getX();
                    } else {
                        return 0.0;
                    }
                }
            )
            
        );
        

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.start().whileTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).andThen(()->drivetrain.resetLatestHeading()));
        //REZERO Gyro

        // joystick.b().onTrue(Commands.runOnce(() ->
        //     m_ShooterSubsystem.setState(
        //         m_ShooterSubsystem.getState() == ShooterStates.PREPASS
        //             ? ShooterStates.IDLE
        //             : ShooterStates.PREPASS
        //     ).schedule()
        // ));

        joystick.b().onTrue(Commands.parallel(
            m_HopperSubsystem.setState(HopperStates.FEED),
            m_ShooterSubsystem.setState(ShooterStates.FORCEPASS)
        ));
        joystick.b().onFalse(Commands.runOnce(() -> {
            m_HopperSubsystem.setState(HopperStates.OFF).schedule();
            m_ShooterSubsystem.setState(ShooterStates.IDLE).schedule();
        }));

        joystick.a().whileTrue(
            drivetrain.SwerveControllerDrive(
                ()-> new Pose2d(3, 4, new Rotation2d()), 
                ()->modifiedDriveInput.getX(), 
                ()->modifiedDriveInput.getY(),
                null, 
                null)
        );
        
        joystick.y().onTrue(Commands.runOnce(() ->
            m_ShooterSubsystem.setState(
                m_ShooterSubsystem.getState() == ShooterStates.PRESHOOT
                    ? ShooterStates.IDLE
                    : ShooterStates.PRESHOOT
            ).schedule()
        ));

        joystick.x().onTrue(Commands.parallel(
            m_HopperSubsystem.setState(HopperStates.FEED),
            m_ShooterSubsystem.setState(ShooterStates.FORCESHOOT)
        ));
        joystick.x().onFalse(Commands.runOnce(() -> {
            m_HopperSubsystem.setState(HopperStates.OFF).schedule();
            m_ShooterSubsystem.setState(ShooterStates.IDLE).schedule();
        }));

        // Aiming
        Command aiming = m_SuperStructure.firingCommand(ShooterStates.AIMING, HopperStates.OFF, DriveStates.AIMING);

        // Shooting
        Command shooting = m_SuperStructure.firingCommand(ShooterStates.SHOOTING, HopperStates.FEED, DriveStates.AIMING);
        Command rotate = drivetrain.SwerveControllerDrive(
                    null, 
                    ()->modifiedDriveInput.getX(), 
                    ()->modifiedDriveInput.getY(),
                    ()->Rotation2d.fromRadians(ShotCalculator.getInstance().getParameters().robotHeadingRadians()), 
                    null);

        Command lockDrive =  new SequentialCommandGroup(
                new WaitCommand(0.01), // Wait 200ms before braking
                    drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
                ).until(()->!enterXLock());

    

        joystick.rightTrigger().or(joystick.leftTrigger())
                    .whileTrue(
                        new RepeatCommand(
                        Commands.deferredProxy(()->{
                            boolean lock = enterXLock();
                            if (lock){
                                return lockDrive;
                            } else{
                                return rotate;
                            }
                        }
                        )
                    ).finallyDo((interrupted) -> drivetrain.SwerveControllerDrive(null, ()->modifiedDriveInput.getX(), ()->modifiedDriveInput.getY(),null,()-> modifiedRotInput.getX()).andThen(()->drivetrain.resetLatestHeading())));

        joystick.rightTrigger()
            .whileTrue(aiming);
        

        joystick.leftTrigger()
            .whileTrue(shooting);

        joystick.povLeft().or(joystick.povRight())
            .whileTrue(drivetrain.SwerveControllerDrive(
                null, 
                ()->modifiedDriveInput.getX(), 
                ()->modifiedDriveInput.getY(),
                null, 
                null)
        );

        joystick.povLeft().whileTrue(aiming);
        joystick.povRight().whileTrue(shooting);

        //intake
        joystick.leftBumper().onTrue(
            Commands.defer(() -> {
                if (m_IntakeSubsystem.isDeployed()) {
                    return m_SuperStructure.intakeRequest(IntakeStates.HOME);
                } else {
                    return m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL);
                }
            }, Set.of(m_IntakeSubsystem, m_SuperStructure)) 
        );


        joystick.rightBumper().whileTrue(
                m_SuperStructure.intaking(IntakeRollerState.INTAKE_ACTIVE)
        );

        // joystick.povDown().whileTrue(
        //     m_SuperStructure.unjam(ShooterStates.UNJAM)
        // );

        joystick.back().onTrue(m_ShooterSubsystem.setState(ShooterStates.REZERO));

        Command outtake = m_SuperStructure.outtakeCommand(IntakeRollerState.INTAKE_OUTTAKE, HopperStates.OUTTAKE, ShooterStates.UNJAM);

        joystick.povUp().whileTrue(outtake);

        // Run SysId routines when holding back/start and X/Y.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
   
    public boolean enterXLock(){
        double rotError = Math.abs((new Rotation2d(ShotCalculator.getInstance().getParameters().robotHeadingRadians()).minus(drivetrain.getState().Pose.getRotation())).getDegrees());
        SmartDashboard.putNumber("Rot Error", rotError);
        if (rotError < 5.0){
            if (rotError < 2.5){
                isLock = true; //lock drive
            } else{
                //do nothing
            }
        } else{
            isLock = false;
        }
        if (!(Math.abs(joystick.getLeftX()) < driveConstants.deadband &&
            Math.abs(joystick.getLeftY()) < driveConstants.deadband &&
            Math.abs(joystick.getRightX()) < driveConstants.deadband)){
                isLock = false;
            }
        SmartDashboard.putBoolean("xlock?", isLock);
        return isLock;
    }

    private void updateLEDState() {
        LEDState desired;
        if (DriverStation.isDisabled()) {
            desired = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? LEDState.DISABLED_RED
                : LEDState.DISABLED_BLUE;
        } else if (m_ShooterSubsystem.getState() == ShooterStates.AIMING && m_SuperStructure.readyToShoot()) {
            desired = LEDState.READY_GREEN;
        } 
        else if (ShotCalculator.getInstance().getParameters().isPassing() && m_SuperStructure.readyToShoot()) {
            desired = LEDState.PASSING_YELLOW_FLASH;
        }else if (m_IntakeSubsystem.getState() == IntakeStates.INTAKE_DEPLOYED) {
            desired = LEDState.IDLE_PINK;
        } else if (m_ShooterSubsystem.getState() == ShooterStates.AIMING) {
            desired = LEDState.ALIGNING_RED_FLASH;
        } else if (DriverStation.isTeleop()){
            desired = LEDState.IDLE_WHITE;
        } else{
            desired = LEDState.IDLE_PINK;
        }
        leds.setState(desired);
    }

    StructArrayPublisher<Pose2d> waypointPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("AutoWaypoints", Pose2d.struct).publish();
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final static Map<positions, Pose2d> waypoints = new HashMap<>();
        public enum positions {StartLeft, ScoreTopLeft, PastLeftTrenchLittle, PastLeftTrenchFar, LeftSideCenterBallOne, StartRight, PastRightTrenchLittle, PastRightTrenchFar, RightSideCenterBallOne, RightScore, Outpost, StartCenter, CenterHangFar, CenterHangNear,LeftTrenchScore,LeftSideCenterBallTwo,LeftCenterHubFar,LeftSideCenterTrench,RightSideCenterHub,RightSideCenterBallTwo,PastLeftBump,LeftBumpScore, LeftSideCenterTransition,RightSideCenterTransition,PastLeftTrenchFarTwo,PastLeftTrenchFarThree,centerStart,LeftCenterHubNear,DepotRight,DepotLeft,DepotScore,DepotNearOne,DepotNearTwo,DepotIntakeOne,DepotIntakeTwo,PastLeftTrenchFarFour,BeforeTrenchLeft,LeftBumpFar,LeftBumpNear,BeforeLeftBump,PastLeftTrenchDiagOne,LeftCenterHubNearTwo,LeftSideCenterBallThree,PastLeftTrenchFarFive,LeftSideBumpTransition,LeftSideCenterBallFour,LeftSideCenterBallFive,PastLeftTrenchFarSix,LeftSideCenterBallSix,PastLeftBumpTwo,LeftSideCenterBallSeven,TowerScore,PastLeftTrenchDiagTwo,LeftSideCenterBallEight}
        // 1. Define Waypoints
        public void generateAutoWaypoints() {
            waypoints.put(positions.StartLeft, new Pose2d(4.0, 7.4, Rotation2d.fromDegrees(0)));
            waypoints.put(positions.ScoreTopLeft, new Pose2d(3.3,6.75, Rotation2d.fromDegrees(-45))); //push into wall?
            waypoints.put(positions.PastLeftTrenchLittle, new Pose2d(6.25, 7.3, Rotation2d.fromDegrees(0)));
            waypoints.put(positions.PastLeftTrenchFar, new Pose2d(8.0, 7, Rotation2d.fromDegrees(90)));
            waypoints.put(positions.PastLeftTrenchFarThree, new Pose2d(7, 7, Rotation2d.fromDegrees(90)));

            waypoints.put(positions.PastLeftTrenchFarTwo, new Pose2d(5.95, 7.3, Rotation2d.fromDegrees(0)));
            waypoints.put(positions.PastLeftTrenchFarFour, new Pose2d(5.93, 7.0, Rotation2d.fromDegrees(-70)));
            waypoints.put(positions.PastLeftTrenchFarFive, new Pose2d(5.7, 7.3, Rotation2d.fromDegrees(180)));
            waypoints.put(positions.PastLeftTrenchFarSix, new Pose2d(6.8, 7, Rotation2d.fromDegrees(120)));
            
            waypoints.put(positions.PastLeftTrenchDiagOne, new Pose2d(8.0, 6.7, Rotation2d.fromDegrees(90)));

            waypoints.put(positions.PastLeftTrenchDiagTwo, new Pose2d(7.7, 6.7, Rotation2d.fromDegrees(90)));
            waypoints.put(positions.LeftSideCenterBallEight, new Pose2d(7.8,4.81, Rotation2d.fromDegrees(90)));
            
            waypoints.put(positions.LeftSideCenterBallOne, new Pose2d(8.1,4.81, Rotation2d.fromDegrees(90)));
            waypoints.put(positions.StartRight, new Pose2d(3.5,0.6, Rotation2d.fromDegrees(0))); 
            waypoints.put(positions.PastRightTrenchLittle, new Pose2d(6,0.76, Rotation2d.fromDegrees(0))); 
            waypoints.put(positions.PastRightTrenchFar, new Pose2d(7.75,0.76, Rotation2d.fromDegrees(-90))); 
            waypoints.put(positions.RightSideCenterBallOne, new Pose2d(7.8,3.5, Rotation2d.fromDegrees(-90)));
            waypoints.put(positions.RightScore, new Pose2d(3.3,1.25, Rotation2d.fromDegrees(45))); //push?
            waypoints.put(positions.Outpost, new Pose2d(1,0.67, Rotation2d.fromDegrees(0))); 
            waypoints.put(positions.StartCenter, new Pose2d(2.5,4.035, Rotation2d.fromDegrees(0)));
            waypoints.put(positions.CenterHangFar, new Pose2d(2.5,5.0, Rotation2d.fromDegrees(-15)));
            waypoints.put(positions.CenterHangNear, new Pose2d(1.3,3.5, Rotation2d.fromDegrees(0)));
            waypoints.put(positions.LeftTrenchScore, new Pose2d(3.25,7.3, Rotation2d.fromDegrees(120))); //this one
            waypoints.put(positions.LeftSideCenterBallTwo, new Pose2d(7,4.5, Rotation2d.fromDegrees(50)));
            waypoints.put(positions.LeftCenterHubFar, new Pose2d(6.1,4.5, Rotation2d.fromDegrees(-90)));
            waypoints.put(positions.LeftSideCenterTrench, new Pose2d(5.5,6, Rotation2d.fromDegrees(-90)));
            waypoints.put(positions.RightSideCenterHub, new Pose2d(5.75,3.5, Rotation2d.fromDegrees(90)));
            waypoints.put(positions.RightSideCenterBallTwo, new Pose2d(7,3.5, Rotation2d.fromDegrees(-90))); 
            waypoints.put(positions.PastLeftBump, new Pose2d(6,5.3, Rotation2d.fromDegrees(-40))); 
            waypoints.put(positions.PastLeftBumpTwo, new Pose2d(6.85,5.35, Rotation2d.fromDegrees(-90))); 
            waypoints.put(positions.LeftBumpScore, new Pose2d(2.5,5.5, Rotation2d.fromDegrees(-42))); 
            waypoints.put(positions.LeftSideCenterTransition, new Pose2d(6.5,4.5, Rotation2d.fromDegrees(0))); 
            waypoints.put(positions.RightSideCenterTransition, new Pose2d(6.5,3.5, Rotation2d.fromDegrees(0))); 
            waypoints.put(positions.centerStart, new Pose2d(3.5,4,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.LeftCenterHubNear, new Pose2d(5.85,4.5,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.DepotRight, new Pose2d(0.8,4.8 ,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.DepotLeft, new Pose2d(0.8,6.3,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.DepotScore, new Pose2d(1.6,5.96,Rotation2d.fromDegrees(-20)));
            waypoints.put(positions.DepotNearOne, new Pose2d(1.4,7.0,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.DepotNearTwo, new Pose2d(1.4,7.4,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.DepotIntakeOne, new Pose2d(0.8,7.0,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.DepotIntakeTwo, new Pose2d(0.8,7.7,Rotation2d.fromDegrees(0)));
            waypoints.put(positions.BeforeTrenchLeft, new Pose2d(3.1, 7.4, Rotation2d.fromDegrees(180)));
            waypoints.put(positions.LeftBumpFar, new Pose2d(7.5, 5.5, Rotation2d.fromDegrees(-60)));
            waypoints.put(positions.LeftBumpNear, new Pose2d(6, 5.6, Rotation2d.fromDegrees(-20)));
            waypoints.put(positions.BeforeLeftBump, new Pose2d(3.0, 5.1, Rotation2d.fromDegrees(-40)));
            waypoints.put(positions.LeftCenterHubNearTwo, new Pose2d(5.85,4.81,Rotation2d.fromDegrees(90)));
            waypoints.put(positions.LeftSideCenterBallThree, new Pose2d(8.1,4.5, Rotation2d.fromDegrees(-90)));
            waypoints.put(positions.LeftSideBumpTransition, new Pose2d(6.05,4.8, Rotation2d.fromDegrees(0))); 
            waypoints.put(positions.LeftSideCenterBallFour, new Pose2d(7,2.8, Rotation2d.fromDegrees(90)));
            waypoints.put(positions.LeftSideCenterBallFive, new Pose2d(7.7,5.4, Rotation2d.fromDegrees(100)));
            waypoints.put(positions.LeftSideCenterBallSix, new Pose2d(7.6,4.9, Rotation2d.fromDegrees(45)));
            waypoints.put(positions.LeftSideCenterBallSeven, new Pose2d(7.6,5.3, Rotation2d.fromDegrees(110)));
            waypoints.put(positions.TowerScore, new Pose2d(1.6,5.3,Rotation2d.fromDegrees(-13)));
            
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                waypoints.replaceAll((pos, pose) -> FlippingUtil.flipFieldPose(pose));
            }
            waypointPublisher.set(waypoints.values().toArray(new Pose2d[0]));
        }

        public Command publishWaypoints(){
            return new InstantCommand(()->waypointPublisher.set(waypoints.values().toArray(new Pose2d[0])));
        }
    
    
    
        // 2. Helper to create a drive command with the error check 
        private Command driveTo(positions pos, double errorMargin) {
            return drivetrain.SwerveControllerDrive(()->waypoints.get(pos), null, null, null, null)
                    .until(() -> drivetrain.getDistanceError() < errorMargin);
        }
    
        public Command driveToWithAngle(positions pos, double errorMargin, DoubleSupplier angle){
            return drivetrain.SwerveControllerDrive(()->waypoints.get(pos), null,null,()->Rotation2d.fromRadians(angle.getAsDouble()),null)
                .until(() -> drivetrain.getDistanceError() < errorMargin);
        }
    
        public static Command getMirrorAllWaypointsCommand() {
            return new InstantCommand(() -> {
                waypoints.forEach((posKey, originalPose) -> {
                    Pose2d mirroredPose = new Pose2d(
                        originalPose.getX(), 
                        K_FIELD_WIDTH - originalPose.getY(), 
                        originalPose.getRotation().unaryMinus()
                    );
                    waypoints.replace(posKey, mirroredPose);
                });
            });
        }

    public Command firingAutonCommand(positions pos){
        return new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                m_SuperStructure.autonFireCommand(ShooterStates.SHOOTING, HopperStates.FEED).repeatedly().withTimeout(1),
                m_SuperStructure.autonFireCommand(ShooterStates.SHOOTING, HopperStates.FEED).repeatedly().until(m_ShooterSubsystem.isBallDetected()).withTimeout(2)
            ).onlyIf(()->drivetrain.getTimeSinceLastTagSeen() < 0.3),
            new SequentialCommandGroup(
                drivetrain.SwerveControllerDrive(()->waypoints.get(pos), null, null, ()->Rotation2d.fromRadians(ShotCalculator.getInstance().getParameters().robotHeadingRadians()), null).until(()->(drivetrain.getDistanceError() < 0.05 && Units.radiansToDegrees(drivetrain.getRotationalError()) < 1)),
                drivetrain.SwerveControllerDrive(null, null, null, ()->Rotation2d.fromRadians(ShotCalculator.getInstance().getParameters().robotHeadingRadians()), null)
            )
            );
            }   

    public Command idleShooter(){
        return m_SuperStructure.autonEndFireCommand().withTimeout(0);
    }

    public Command aimingAutonCommand(positions pos){   
        return new ParallelDeadlineGroup(
                drivetrain.SwerveControllerDrive(()->waypoints.get(pos), null, null, ()->Rotation2d.fromRadians(ShotCalculator.getInstance().getParameters().robotHeadingRadians()), null)
                .until(()->(drivetrain.getDistanceError() < 0.1 && Units.radiansToDegrees(drivetrain.getRotationalError()) < 1 && drivetrain.getTimeSinceLastTagSeen() < 0.1)),
                m_SuperStructure.autonShooterRequest(ShooterStates.AIMING).repeatedly().onlyIf(()->drivetrain.getTimeSinceLastTagSeen() < 0.1)
            ).withTimeout(1.0);
    }

    public Command spinUpShooter(){
        return m_SuperStructure.autonFireCommand(ShooterStates.SPINUP, HopperStates.OFF).withTimeout(0);
    }

    public Command driveToWayPoint(positions pos, double errorMargin, double velocity){
        Command driveCommand = drivetrain.SwerveWaypointDrive(()->waypoints.get(pos), null, ()->velocity)
                .until(() -> drivetrain.getDistanceError() < errorMargin);

        return driveCommand;
    }

    public Command driveToWayPointWithAngle(positions pos, double errorMargin, DoubleSupplier angle,  Double velocity){
        return drivetrain.SwerveWaypointDrive(()->waypoints.get(pos), ()->Rotation2d.fromRadians(angle.getAsDouble()), ()->velocity)
                .until(() -> drivetrain.getDistanceError() < errorMargin);
    }

    public Command driveToIntake(positions pos, double errorMargin){
        return new ParallelDeadlineGroup(
            drivetrain.SwerveControllerDrive(()->waypoints.get(pos), null, null, null, null).until(() -> drivetrain.getDistanceError() < errorMargin),
            m_SuperStructure.autonIntake(IntakeRollerState.INTAKE_AUTO)
        );
    }

    public Command endIntake(){
        return m_SuperStructure.autonEndIntake().withTimeout(0);
    }

    public Command driveToIntakeWithAngle(positions pos, double errorMargin, DoubleSupplier angle){
        return new ParallelDeadlineGroup(
            drivetrain.SwerveControllerDrive(()->waypoints.get(pos), null,null,()->Rotation2d.fromRadians(angle.getAsDouble()),null)
            .until(() -> drivetrain.getDistanceError() < errorMargin),
            m_SuperStructure.autonIntake(IntakeRollerState.INTAKE_AUTO)
        );
    }

    public Command driveToIntakeWayPoint(positions pos, double errorMargin, double velocity){
        return new ParallelDeadlineGroup(
            drivetrain.SwerveWaypointDrive(()->waypoints.get(pos), null, ()->velocity)
                .until(() -> drivetrain.getDistanceError() < errorMargin),
            m_SuperStructure.autonIntake(IntakeRollerState.INTAKE_AUTO)
        );
    }

    public Command driveToIntakeWayPointWithAngle(positions pos, double errorMargin, DoubleSupplier angle, double velocity){
        return new ParallelDeadlineGroup(
            drivetrain.SwerveWaypointDrive(()->waypoints.get(pos), ()->Rotation2d.fromRadians(angle.getAsDouble()), ()->velocity)
                .until(() -> drivetrain.getDistanceError() < errorMargin),
            m_SuperStructure.autonIntake(IntakeRollerState.INTAKE_AUTO)
        );
    }

    public DoubleSupplier faceAngle(double degrees) {
        return () -> {
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            return Units.degreesToRadians(isRed ? degrees+180 : degrees);
        };
    }

    private Command leftSideDoubleSwipe(){
        return
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    drivetrain.runOnce(()-> drivetrain.resetPose(waypoints.get(positions.StartLeft))), //seed starting pos
                    m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
                ),
                m_SuperStructure.shooterRequest(ShooterStates.IDLE).withTimeout(0),
                driveTo(positions.StartLeft, 1.0), //ensure team isnt a clown
                driveToWayPoint(positions.PastLeftTrenchLittle, 0.2, 5.3),
                driveToIntakeWayPoint(positions.PastLeftTrenchFar, 0.2,3),
                driveToIntakeWayPoint(positions.LeftSideCenterBallOne, 0.1, 2.5), //Intaked balls returning
                endIntake(),
                spinUpShooter(),
                driveToWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(0)), //increase to 5.0 if not hit wall hard
                driveToWayPointWithAngle(positions.StartLeft, 0.3,faceAngle(0), 5.0), //3.5 vel
                aimingAutonCommand(positions.LeftTrenchScore),
                firingAutonCommand(positions.LeftTrenchScore),
                idleShooter(),
                //second Swipe
                driveToWayPointWithAngle(positions.StartLeft, 0.05, faceAngle(0), 5.0),         //0.1
                // driveToWayPoint(positions.PastLeftTrenchLittle, 0.1, 5.0),
                driveToWayPointWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(0), 4.0),
                driveToIntakeWayPoint(positions.PastLeftTrenchFarThree, 0.2, 3.0),
                driveToIntakeWayPoint(positions.LeftSideCenterBallTwo, 0.15, 2.5),
                driveToIntakeWayPoint(positions.LeftSideCenterTransition, 0.05, 4.0), 
                driveToIntakeWayPoint(positions.LeftCenterHubNear, 0.05,2.5),
                driveToIntakeWayPoint(positions.LeftCenterHubFar, 0.1,4.0), //may have to lower (in case slide)
                driveToIntakeWayPoint(positions.PastLeftTrenchFarFour, 0.2, 3.0), 
                driveToIntakeWayPointWithAngle(positions.PastLeftTrenchFarTwo, 0.15, faceAngle(0), 3.0), 
                endIntake(),
                spinUpShooter(),
                driveToWayPointWithAngle(positions.StartLeft, 0.05, faceAngle(0), 5.0), //3.5 vel
                aimingAutonCommand(positions.LeftTrenchScore),
                firingAutonCommand(positions.LeftTrenchScore),
                idleShooter()
        );
    }

    private Command firstSwipeBumpFar(){
        return
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    drivetrain.runOnce(()-> drivetrain.resetPose(waypoints.get(positions.StartLeft))), //seed starting pos
                    m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
                ),
                m_SuperStructure.shooterRequest(ShooterStates.IDLE).withTimeout(0),
                driveTo(positions.StartLeft, 1.0), //ensure team isnt a clown
                driveToWayPoint(positions.PastLeftTrenchFarTwo, 0.3, 5.3),
                driveToIntakeWayPoint(positions.PastLeftTrenchDiagOne, 0.3,5.3),
                driveToIntakeWayPoint(positions.LeftSideCenterBallOne, 0.1, 2.5), //Intaked balls returning
                spinUpShooter(),
                driveToIntakeWayPoint(positions.PastLeftBump, 0.2, 4.5), 
                endIntake(),
                driveToWayPoint(positions.BeforeLeftBump, 0.1, 4.5),
                driveTo(positions.LeftBumpScore, 0.3), //3.5 vel
                aimingAutonCommand(positions.LeftBumpScore),
                firingAutonCommand(positions.LeftBumpScore),
                idleShooter()
        );
    }

    private Command firstSwipeBumpNear(){
        return
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    drivetrain.runOnce(()-> drivetrain.resetPose(waypoints.get(positions.StartLeft))), //seed starting pos
                    m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
                ),
                m_SuperStructure.shooterRequest(ShooterStates.IDLE).withTimeout(0),
                driveTo(positions.StartLeft, 1.0), //ensure team isnt a clown
                driveToWayPoint(positions.PastLeftTrenchFarTwo, 0.3, 5.3),
                driveToIntakeWayPoint(positions.PastLeftTrenchDiagTwo, 0.3,4.5),
                driveToIntakeWayPoint(positions.LeftSideCenterBallEight, 0.1, 2.5), //Intaked balls returning
                spinUpShooter(),
                driveToIntakeWayPoint(positions.PastLeftBump, 0.2, 4.5), 
                endIntake(),
                driveToWayPoint(positions.BeforeLeftBump, 0.5, 4.5),
                driveTo(positions.LeftBumpScore, 0.1), 
                aimingAutonCommand(positions.LeftBumpScore),
                firingAutonCommand(positions.LeftBumpScore),
                idleShooter()
        );
    }

    private Command secondSwipeBumpTrenchFar(){
        return
            new SequentialCommandGroup(
                //second Swipe
                driveToWayPoint(positions.BeforeTrenchLeft, 0.1, 3.0),
                driveToIntakeWayPointWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(180), 4.0),
                driveToIntakeWayPoint(positions.PastLeftTrenchFarSix, 0.2, 3.0),
                // driveToIntakeWayPoint(positions.LeftSideCenterBallFive, 0.2, 3.5),
                driveToIntakeWayPoint(positions.LeftSideCenterBallSeven, 0.2, 3.5),
                driveToIntakeWayPoint(positions.LeftSideCenterBallSix, 0.2, 3.5),
                driveToIntakeWayPoint(positions.LeftSideBumpTransition, 0.3, 3.5), 
                spinUpShooter(),
                driveToIntakeWayPoint(positions.PastLeftBumpTwo, 0.05, 3.0), //increase to 5.0 if not hit wall hard
                driveToIntakeWayPoint(positions.PastLeftBump, 0.2, 5), //increase to 5.0 if not hit wall hard
                endIntake(),
                driveToWayPoint(positions.BeforeLeftBump, 0.2, 4.5),
                driveTo(positions.LeftBumpScore, 0.1), //3.5 vel
                aimingAutonCommand(positions.LeftBumpScore),
                firingAutonCommand(positions.LeftBumpScore),
                idleShooter()
            );
    }

    private Command secondSwipeBumpModified(){
        return
            new SequentialCommandGroup(
                //second Swipe
                driveToWayPoint(positions.BeforeTrenchLeft, 0.1, 3.0),
                driveToIntakeWayPointWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(180), 4.0),
                driveToIntakeWayPoint(positions.PastLeftTrenchFarSix, 0.2, 3.0),
                // driveToIntakeWayPoint(positions.LeftSideCenterBallFive, 0.2, 3.5),
                driveToIntakeWayPoint(positions.LeftSideCenterBallSeven, 0.2, 3.5),
                driveToIntakeWayPoint(positions.LeftSideCenterBallSix, 0.2, 3.5),
                driveToIntakeWayPoint(positions.LeftSideBumpTransition, 0.3, 3.5), 
                spinUpShooter(),
                // driveToIntakeWayPoint(positions.PastLeftBumpTwo, 0.05, 3.0), 
                driveToIntakeWayPoint(positions.PastLeftBump, 0.2, 2.5), 
                driveToWayPoint(positions.BeforeLeftBump, 0.2, 5.0),
                driveToWayPoint(positions.LeftBumpScore, 0.1, 3.0), //3.5 vel
                aimingAutonCommand(positions.LeftBumpScore),
                firingAutonCommand(positions.LeftBumpScore),
                idleShooter()
            );
    }

    private Command secondSwipeBumpHub(){
        return
            new SequentialCommandGroup(
                //second Swipe
                driveTo(positions.BeforeTrenchLeft, 0.1),
                driveToWayPoint(positions.PastLeftTrenchFarFive, 0.3, 4.0),
                driveToIntakeWayPoint(positions.LeftCenterHubNearTwo, 0.2, 3.0), 
                driveToIntakeWayPointWithAngle(positions.LeftSideCenterTransition, 0.3, faceAngle(180), 5.0), 
                driveToIntakeWayPoint(positions.LeftSideCenterBallThree, 0.3, 5.0),
                driveToIntakeWayPoint(positions.LeftBumpFar, 0.2, 3.0),
                driveToIntakeWayPoint(positions.LeftBumpNear, 0.3, 3.0),
                // driveToIntakeWayPoint(positions.PastLeftBump, 0.2, 3.0),
                endIntake(),
                spinUpShooter(),
                driveToWayPoint(positions.BeforeLeftBump, 0.1, 4.5),
                driveTo(positions.LeftBumpScore, 0.3), //3.5 vel
                aimingAutonCommand(positions.LeftBumpScore),
                firingAutonCommand(positions.LeftBumpScore),
                idleShooter()
            );
    }

    private Command secondSwipeBumpCross(){
        return
            new SequentialCommandGroup(
                //second Swipe
                driveTo(positions.BeforeTrenchLeft, 0.1),
                driveToWayPointWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(180), 4.0),
                driveToIntakeWayPoint(positions.PastLeftTrenchFarThree, 0.2, 3.0),
                driveToIntakeWayPoint(positions.LeftSideCenterBallFour, 0.15, 2.5),
                getMirrorAllWaypointsCommand(),
                publishWaypoints(),
                driveToIntakeWayPoint(positions.PastLeftBump, 0.2, 5.0), //increase to 5.0 if not hit wall hard
                endIntake(),
                spinUpShooter(),
                driveToWayPoint(positions.LeftBumpScore, 0.3, 5.0), //3.5 vel
                aimingAutonCommand(positions.LeftBumpScore),
                firingAutonCommand(positions.LeftBumpScore),
                idleShooter()
            );
    }

    private Command bumpFollow(){
        return
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    drivetrain.runOnce(()-> drivetrain.resetPose(waypoints.get(positions.StartLeft))), //seed starting pos
                    m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
                ),
                new WaitCommand(4),
                driveTo(positions.StartLeft, 1.0), 
                driveToIntakeWayPoint(positions.PastLeftTrenchLittle, 0.2, 4.0), //4.0
                driveToIntakeWayPoint(positions.PastLeftTrenchFarSix, 0.2, 3.0), //3.0
                driveToIntakeWayPoint(positions.LeftSideCenterBallSeven, 0.2, 3.5), //3.5
                driveToIntakeWayPoint(positions.LeftSideCenterBallSix, 0.2, 3.5), //3.5
                driveToIntakeWayPoint(positions.LeftSideBumpTransition, 0.3, 3.5), //3.5
                spinUpShooter(),
                driveToIntakeWayPoint(positions.PastLeftBumpTwo, 0.05, 3.0), //increase to 5.0 if not hit wall hard
                endIntake(),
                // new WaitCommand(2), // wait command does a funny
                driveToWayPoint(positions.BeforeLeftBump, 0.1, 5.3),
                driveToWayPoint(positions.TowerScore, 0.3, 2.5), //3.5 vel
                aimingAutonCommand(positions.TowerScore),
                firingAutonCommand(positions.TowerScore),
                idleShooter()
            );
    }

    private Command addTrenchReturnCenter(){
        return 
            new SequentialCommandGroup(
                //return to center    
                driveToWayPointWithAngle(positions.StartLeft, 0.1, faceAngle(0), 5.0),
                driveToWayPointWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(0), 4.0),
                driveToWayPoint(positions.PastLeftTrenchFar, 0.2, 5.0),
                driveToIntakeWayPoint(positions.LeftSideCenterBallOne, 0.15, 3.5),
                endIntake()
            );
    }

    private Command addBumpReturnCenter(){
        return 
            new SequentialCommandGroup(
                //return to center    
                driveTo(positions.BeforeTrenchLeft, 0.1),
                driveToWayPointWithAngle(positions.PastLeftTrenchLittle, 0.2, faceAngle(180), 4.0),
                driveToIntakeWayPoint(positions.PastLeftTrenchFar, 0.2, 5.0),
                driveToIntake(positions.LeftSideCenterBallOne, 0.1),
                endIntake()
            );
    }

    private Command addDepot(){
        return
            new SequentialCommandGroup(
                // drive to depot
                endIntake(),
                driveToWayPoint(positions.DepotRight, 0.05, 1.5), 
                driveToWayPoint(positions.DepotLeft, 0.05, 1.5), 
                driveToWayPoint(positions.DepotNearOne, 0.05, 2.5), 
                driveToIntakeWayPoint(positions.DepotIntakeOne, 0.05, 1.5),
                driveToWayPoint(positions.DepotNearOne, 0.05, 2.5), 
                driveToIntakeWayPoint(positions.DepotNearTwo, 0.05, 2.5),
                driveToIntakeWayPoint(positions.DepotIntakeTwo, 0.05, 1.5),
                endIntake(),
                driveToWayPoint(positions.DepotNearTwo, 0.1, 2.5),
                driveToWayPoint(positions.DepotScore, 0.2, 2.5),
                aimingAutonCommand(positions.DepotScore), 
                firingAutonCommand(positions.DepotScore),
                idleShooter()
        );
    }

    private Command preload(){
        return 
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    drivetrain.runOnce(()-> drivetrain.resetPose(waypoints.get(positions.centerStart))),
                    m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
                ),            
                m_SuperStructure.shooterRequest(ShooterStates.IDLE).withTimeout(0),
                spinUpShooter(),
                driveTo(positions.CenterHangFar, 0.05),
                aimingAutonCommand(positions.CenterHangFar),
                firingAutonCommand(positions.CenterHangFar),
                idleShooter()
            );
    }

    // 3. Setup Chooser (Easily add more here)
    private void createAutos() {
        m_chooser.setDefaultOption("No Auto", new SequentialCommandGroup(
            drivetrain.runOnce(drivetrain::seedFieldCentric).andThen(()->drivetrain.resetLatestHeading())
        ));

        m_chooser.addOption("Left Center Double Swipe", new SequentialCommandGroup(
            leftSideDoubleSwipe(),
            addTrenchReturnCenter()
        ));

        // m_chooser.addOption("Right Center > Depot", new SequentialCommandGroup(
        //     //one    
        //     new ParallelDeadlineGroup(
        //             drivetrain.runOnce(()-> drivetrain.resetTranslation(waypoints.get(positions.StartRight).getTranslation())),
        //             m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
        //     ),   
        //     driveTo(positions.StartRight, 0.1),
        //     driveToWayPoint(positions.PastRightTrenchLittle, 0.1, 4.0),
        //     driveToWayPoint(positions.PastRightTrenchFar, 0.1, 4.0), //may need superfar trench
        //     driveToIntake(positions.RightSideCenterBallOne, 0.1), 
        //         //return
        //     driveToWayPoint(positions.PastRightTrenchLittle, 0.05, 3.0),
        //     driveToWayPointWithAngle(positions.StartRight, 0.05, ()->ShotCalculator.getInstance().getParameters().robotHeadingRadians(), 3.0), 
        //     aimingAutonCommand(positions.RightScore),
        //     firingAutonCommand(positions.RightScore),
        //     //two
        //     driveToWayPoint(positions.Outpost, 0.05, 2.5), 
        //     new WaitCommand(2),
        //     aimingAutonCommand(positions.RightScore),
        //     firingAutonCommand(positions.RightScore)
        // ));

        m_chooser.addOption("Right Double Swipe", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            leftSideDoubleSwipe(),
            addTrenchReturnCenter()
        ));

        // m_chooser.addOption("Left Depot", new SequentialCommandGroup(
        //     addDepot()
        // ));

        // m_chooser.addOption("Left Bump in out", new SequentialCommandGroup(
        //     firstSwipeBump(),
        //     secondSwipeBumpHub(),
        //     addBumpReturnCenter()
        // ));

        // m_chooser.addOption("Right Bump in out", new SequentialCommandGroup(
        //     getMirrorAllWaypointsCommand(),
        //     publishWaypoints(),
        //     firstSwipeBump(),
        //     secondSwipeBumpHub(),
        //     addBumpReturnCenter()
        // ));

        m_chooser.addOption("Left Bump FAR MODIFIED", new SequentialCommandGroup(
            firstSwipeBumpFar(),
            secondSwipeBumpModified(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Right Bump FAR MODIFIED", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            firstSwipeBumpFar(),
            secondSwipeBumpModified(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Left Bump NEAR MODIFIED", new SequentialCommandGroup(
            firstSwipeBumpNear(),
            secondSwipeBumpModified(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Right Bump NEAR MODIFIED", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            firstSwipeBumpNear(),
            secondSwipeBumpModified(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Left Bump FAR out in", new SequentialCommandGroup(
            firstSwipeBumpFar(),
            secondSwipeBumpTrenchFar(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Right Bump FAR out in", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            firstSwipeBumpFar(),
            secondSwipeBumpTrenchFar(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Left Bump NEAR out in", new SequentialCommandGroup(
            firstSwipeBumpNear(),
            secondSwipeBumpTrenchFar(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Right Bump NEAR out in", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            firstSwipeBumpNear(),
            secondSwipeBumpTrenchFar(),
            addBumpReturnCenter()
        ));

        m_chooser.addOption("Left Bump cross", new SequentialCommandGroup(
            firstSwipeBumpFar(),
            secondSwipeBumpCross()
        ));

        m_chooser.addOption("Right Bump cross", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            firstSwipeBumpFar(),
            secondSwipeBumpCross()
        ));

        m_chooser.addOption("preload", new SequentialCommandGroup(
            preload(),
            addDepot()
        ));

        m_chooser.addOption("left bump follow", new SequentialCommandGroup(
            bumpFollow(),
            addDepot()
        ));

        m_chooser.addOption("right bump follow", new SequentialCommandGroup(
            getMirrorAllWaypointsCommand(),
            publishWaypoints(),
            bumpFollow()
        ));

        // m_chooser.addOption("Test Auto", new SequentialCommandGroup(
        //     new ParallelDeadlineGroup(
        //         drivetrain.runOnce(()-> drivetrain.resetTranslation(waypoints.get(positions.StartLeft).getTranslation())), //seed starting pos
        //         m_SuperStructure.intakeRequest(IntakeStates.INTAKE_TRAVEL)
        //     ),
        //     driveTo(positions.StartLeft, 1.0),
        //     driveToWayPoint(positions.PastLeftTrenchLittle, 0.5,4.0),
        //     driveToWayPoint(positions.PastLeftTrenchFar, 0.2,5.0),
        //     driveTo(positions.LeftSideCenterBallOne, 0.05)
        // ));

        // m_chooser.addOption("Test Right Auto", new SequentialCommandGroup(
        //     driveTo(positions.StartRight, 0.5)
        // ));

        SmartDashboard.putData(m_chooser);
    }


    public Command getAutonomousCommand() {
        generateAutoWaypoints();
        return m_chooser.getSelected();
    }

}
