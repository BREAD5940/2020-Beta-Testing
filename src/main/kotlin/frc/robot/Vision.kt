package frc.robot
import edu.wpi.cscore.VideoMode
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.geometry.Rotation2d
import frc.robot.subsystems.drive.DriveCommand
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.pico
import org.ghrobotics.lib.wrappers.networktables.get
import java.awt.image.PixelGrabber
import java.awt.image.PixelInterleavedSampleModel

    object Vision {
    var prevError = 0.0
    val table = NetworkTableInstance.getDefault().getTable("limelight")
    val txEntry = table.getEntry("tx")
    val tx = {  txEntry.getDouble(0.0) }
    val tLongEntry = table.getEntry("tlong")
    val tlong = { tLongEntry.getDouble(0.0)}
    val gyroCurrent = DriveSubsystem.gyro
    val gyroGo = DriveSubsystem.gyro() + DriveSubsystem.wantedAngle

    class AimAtVisionTarget : FalconCommand(DriveSubsystem) {
        val kp = 0.011
        var currentTx = 0.0

        override fun execute() {
            currentTx = tx.invoke()
            val turn = -currentTx * kp
            DriveSubsystem.arcadeDrive(0.0, turn)
            }
       override fun isFinished(): Boolean {
            return currentTx > 4
        }

    }
    class VisionGoToTarget {
        val tLongWanted = 190.0 //Pixels on screen
        val kp = 0.0
        var currentTLong = 0.0
        var currentTx = 0.0

        fun execute() {
            currentTLong = tlong.invoke()
            val error = tLongWanted - currentTLong
            val forward =  error * 0.0

            currentTx = tx.invoke()
            val turn = -currentTx * 0.01

            DriveSubsystem.arcadeDrive(forward, turn)

        }
    }

}

