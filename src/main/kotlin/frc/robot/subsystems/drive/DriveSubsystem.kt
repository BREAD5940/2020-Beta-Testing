package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source

object DriveSubsystem : FalconWestCoastDrivetrain() {
    override val controller: RamseteController
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val gyro: Source<Rotation2d>
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val kinematics: DifferentialDriveKinematics
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val leftCharacterization: MotorCharacterization<Meter>
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val leftMotor: FalconMotor<Meter>
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val odometry: DifferentialDriveOdometry
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val rightCharacterization: MotorCharacterization<Meter>
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.
    override val rightMotor: FalconMotor<Meter>
        get() = TODO("not implemented") // To change initializer of created properties use File | Settings | File Templates.

    override fun activateEmergency() {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }

    override fun recoverFromEmergency() {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }
}
