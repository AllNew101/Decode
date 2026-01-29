package org.firstinspires.ftc.teamcode.opmode.Calculate;

public class BinarySearch {
    //Binary Search system
    double[] D;
    double right, left;

    public BinarySearch(double[] data){
        this.D = data;
    }
    public int find(int left ,int right, double goal) {
        int mid = (int) Math.floor((left + right) / 2);
        if (D[mid] < goal && goal < D[mid + 1] || D[mid] == goal) {
            return mid;
        }
        else {
            if (goal> D[mid]) {
                return find(mid, right, goal);
            }
            else {
                return find(left, mid, goal);
            }
        }
    }
}
