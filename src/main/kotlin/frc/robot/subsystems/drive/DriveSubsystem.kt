package frc.robot.subsystems.drive

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconDriveHelper
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source
import javax.naming.ldap.Control

object DriveSubsystem : FalconSubsystem() {

    val leftMotor = falconMAX(1, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        outputInverted = true
        brakeMode = true
        with(canSparkMax) {
            setSecondaryCurrentLimit(50.0)
            setSmartCurrentLimit(40)
        }
    }

    val rightMotor = falconMAX(2, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        outputInverted = false
        brakeMode = true
        with(canSparkMax) {
            setSecondaryCurrentLimit(50.0)
            setSmartCurrentLimit(40)
        }
    }

    override fun lateInit() {
        defaultCommand = object : FalconCommand(DriveSubsystem) {
            val helper = FalconDriveHelper()
            override fun execute() {
                val forward = -Controls.driverControllerLowLevel.getY(GenericHID.Hand.kLeft)
                val turn = Controls.driverControllerLowLevel.getX(GenericHID.Hand.kRight)

                val output = helper.curvatureDrive(forward, turn,
                        Controls.driverControllerLowLevel.getBumper(GenericHID.Hand.kRight))

                leftMotor.setDutyCycle(output.first)
                rightMotor.setDutyCycle(output.second)
            }
        }
    }

}
