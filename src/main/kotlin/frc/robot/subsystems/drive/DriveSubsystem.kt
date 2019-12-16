package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import frc.robot.Vision
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source
import java.rmi.activation.ActivationGroupDesc


object DriveSubsystem : FalconWestCoastDrivetrain() {
    lateinit var wantedAngle: Rotation2d
    override val controller = RamseteController(2.0,0.7)
    private val gyro_ = AHRS(SPI.Port.kMXP)
    override val gyro = { (gyro_.yaw * -1).degrees.toRotation2d() }
    val gyroAngle = gyro
    override val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics((26.0).inches.inMeters())
    override val leftCharacterization = SimpleMotorFeedforward(1.49, 2.66, 0.918)
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
    override val odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(kinematics, gyro()).apply {
        resetPosition(Pose2d())
    }
    override val rightCharacterization = SimpleMotorFeedforward(1.49,2.66,0.918)
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

    class GoToGyroAngle() : FalconCommand(DriveSubsystem){
        //TODO tune the pids
        val kP = 0.9
        val kD = 4.95

        val error = (DriveSubsystem.gyro() - wantedAngle).radians
             //lateinit var wantedAngle: Rotation2d
                  var prevError = 0.0

        fun initalize(){
            //TODO ALSO FIX
            Vision.AimAtVisionTarget()
        }
       override  fun execute(){
            val der = (error - prevError) * kD
            val output = error * kP + der
                                        //Forward + turn
            DriveSubsystem.leftMotor.setDutyCycle(DriveCommand.speedSource() -output)
            DriveSubsystem.rightMotor.setDutyCycle(DriveCommand.speedSource() + output)

            prevError = error
        }//he




        fun isfinished() : Boolean{
            return error < 5 //5 degrees error
        }




    }
}
