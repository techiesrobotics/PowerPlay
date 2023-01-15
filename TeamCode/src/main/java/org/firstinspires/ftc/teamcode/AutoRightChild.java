package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRightChild", group = "ConceptBlue")
public class AutoRightChild extends AutoParent{
    public double adjustTurn(double angle){
       return -angle;
    }

    public double adjustTrajectorydistance(double distance){
        return -distance;
    }

    protected void park() {
        if (targetZone == 3) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-47)));
            forward(25);

        } else if (targetZone == 2) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-47)));
        } else if (targetZone == 1) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-47)));
            back(24);

        }
    }
}
