package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source

object DriveSubsystem : FalconWestCoastDrivetrain() {
    override val controller = RamseteController(2.0,0.7)
    val gyro_ = AHRS(SPI.Port.kMXP)
    override val gyro = { (gyro_.yaw * -1).degrees.toRotation2d() }
    override val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics((26.0).inches.inMeters())
    override val leftCharacterization = MotorCharacterization<Meter>(SIUnit(2.66), SIUnit(0.918), SIUnit(1.49))
    public override val leftMotor = FalconSRX(id = 1, model = NativeUnitLengthModel(4096.nativeUnits, 2.inches)).apply { /* this: FalconSRX<NativeUnit> */
        talonSRX.configFactoryDefault()
        talonSRX.config_kP(0,1.78)
        talonSRX.selectProfileSlot(0,0)
        outputInverted = true // TODO Replace me with what you found works for the leftMotor
    }
    val leftFollower = FalconSRX(id = 2, model = NativeUnitLengthModel(4096.nativeUnits, 2.inches)).apply { /* this: FalconSRX<NativeUnit> */
        talonSRX.configFactoryDefault()
        outputInverted = true // TODO Replace me with what you found works for the leftFollower
        follow(leftMotor)
    }
    override val odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(kinematics).apply {
        resetPosition(Pose2d())
    }
    override val rightCharacterization = MotorCharacterization<Meter>(SIUnit(2.66), SIUnit(0.918),SIUnit(1.49))
    public override val rightMotor = FalconSRX(id=3, model=NativeUnitLengthModel(4096.nativeUnits, 2.inches)).apply { /* this: FalconSRX<NativeUnit> */
        talonSRX.configFactoryDefault()
        talonSRX.config_kP(0,1.78)
        talonSRX.selectProfileSlot(0,0)
        outputInverted = false // TODO Replace me with what you found works for the rightMotor
    }
    val rightFollower = FalconSRX(id = 4, model = NativeUnitLengthModel(4096.nativeUnits, 2.inches)).apply { /* this: FalconSRX<NativeUnit> */
        talonSRX.configFactoryDefault()
        outputInverted = false // TODO Replace me with what you found works for the rightFollower
        follow(rightMotor)
    }

    override fun activateEmergency() {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }

    override fun recoverFromEmergency() {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }
    override fun lateInit() {
        defaultCommand = DriveCommand()
    }

}
