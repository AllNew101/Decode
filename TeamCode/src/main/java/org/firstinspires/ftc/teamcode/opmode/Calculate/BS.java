package org.firstinspires.ftc.teamcode.opmode.Calculate;

public class BS {
    //Binary Search system
    double[] D;
    double d, right, left;
    public void set(double[] data ,double goal){
        this.D = data;
        this.d = goal;

    }
    public double find(int left , int right) {
        int mid = (int) Math.floor((left + right) / 2);
        if (D[mid] < d && d < D[mid + 1] || D[mid] == d) {
            return mid;
        }
        else {
            if (d > D[mid]) {
                return find(mid, right);
            }
            else {
                return find(left, mid);
            }
        }
    }
}
