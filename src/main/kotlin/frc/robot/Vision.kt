package frc.robot
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.geometry.Rotation2d
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees

private val Any.degree: Any
    get() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

object Vision {

    val table = NetworkTableInstance.getDefault().getTable("limelight")
        val txAngle = (table.getEntry("tx"))
    //TODO FIX THIS make it invert the signal
            val angle = txAngle.degree as Rotation2d


     class VisionAngle(){
         val gyroCurrent = DriveSubsystem.gyro
         val gyroGo = DriveSubsystem.wantedAngle
         fun initalize() {
             DriveSubsystem.wantedAngle = angle as Rotation2d
                    }

         fun isfinished(): Boolean {
             return angle < 6.degrees
         }



     }
}

