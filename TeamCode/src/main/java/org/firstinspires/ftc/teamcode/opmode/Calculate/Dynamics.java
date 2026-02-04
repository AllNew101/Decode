package org.firstinspires.ftc.teamcode.opmode.Calculate;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.opmode.Calculate.Distance;

public class Dynamics {
    Distance distance = new Distance();
    double a,b,c,ti,t_positive,t_negative,inner;
    double[] dist;

    Vector dis,vb;
    public double time(Vector d, Vector vb, Vector vr){
        a = vr.dot(vr) - vb.dot(vb);
        b = 2 * d.dot(vr);
        c = d.dot(d);
        inner = (b * b) - (4 * a * c);
        t_positive = (b + Math.sqrt(inner)) / (2 * a);
        t_negative = (b - Math.sqrt(inner)) / (2 * a);
        if (inner > 0 && vr.getMagnitude() > 1){
            if (t_positive > 0){return t_positive;}
            else{return t_negative;}
        }
        else{
            return 0;
        }

    }
    public Pose lead(Pose position, Vector vr, double vball, double theta, boolean is_red){
        dist = distance.distance(position.getX(), position.getY(), is_red);
        dis = new Vector(dist[3],Math.toRadians(dist[2]));
        vb = new Vector(vball,theta);
        ti = time(dis,vb,vr);
        Pose lead_XY = new Pose(vr.getXComponent() * ti, vr.getYComponent() * ti);

        return new Pose(position.getX() + lead_XY.getX(), position.getY() + lead_XY.getY() , position.getHeading());
    }

    public double get_t(){return ti;}
    public double get_a(){return a;}
    public double get_b(){return b;}
    public double get_c(){return c;}
    public Vector get_vb(){return vb;}
    public Vector get_dis(){return dis;}
    public double get_dot(Vector Vr){return dis.dot(Vr);}

}
