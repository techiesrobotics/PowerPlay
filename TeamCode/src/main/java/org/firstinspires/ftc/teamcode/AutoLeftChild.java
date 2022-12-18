package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoLeftChild", group = "ConceptBlue")
public class AutoLeftChild extends AutoParent{
    public double adjustTurn(double angle){
       return angle;
    }
    protected void park() {
        if (targetZone == 3) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-137)));
            back(25);

        } else if (targetZone == 2) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-137)));
        } else if (targetZone == 1) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(-137)));
            forward(24);

        }
    }
}
