//
// Created by chaoz on 22/10/17.
//

#include "state_display.h"
//screen.cpp


Screen2::Screen2() {
    colorBlack.val[COLOR_BLUE] = 0;
    colorBlack.val[COLOR_GREEN] = 0;
    colorBlack.val[COLOR_RED] = 0;
    colorOrange.val[COLOR_BLUE] = 0;
    colorOrange.val[COLOR_GREEN] = 140;
    colorOrange.val[COLOR_RED] = 255;
    colorRed.val[COLOR_BLUE] = 0;
    colorRed.val[COLOR_GREEN] = 0;
    colorRed.val[COLOR_RED] = 255;
    colorBlue.val[COLOR_BLUE] = 255;
    colorBlue.val[COLOR_GREEN] = 0;
    colorBlue.val[COLOR_RED] = 0;
    colorGreen.val[COLOR_BLUE] = 0;
    colorGreen.val[COLOR_GREEN] = 255;
    colorGreen.val[COLOR_RED] = 0;

    MyLaserMaxRange = 20000;
    int scaleFactor = 15;
    MyScaleX = MyScaleY = scaleFactor;
    MyRobotPosition = Point((int) (MyLaserMaxRange / MyScaleX),
                            (int) ceil((MyLaserMaxRange + MyLaserOffset) / MyScaleY));
    MyLaserPosition = MyRobotPosition;
    MyLaserPosition.y -= MyLaserOffset / MyScaleY;
    MyBackground = Mat((int) ((MyLaserMaxRange + MyLaserOffset) / MyScaleY),
                       (int) (2 * MyLaserMaxRange / MyScaleX), CV_8UC3, Scalar(255, 255, 255));
    rectangle(MyBackground, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 200, 200), FILLED);
    circle(MyBackground, MyLaserPosition, MyLaserPosition.x, Scalar(0, 255, 0), 1);
    putText(MyBackground, "Max Laser Range", Point(2, 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "FreeSpace", Point(2, 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "Max Laser Range", Point(2, 30), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));

    MyBackground.copyTo(MyImage);
    MySize.width = (int) (2 * MyLaserMaxRange / MyScaleX);
    MySize.height = (int) (MyLaserMaxRange / MyScaleY);
    namedWindow(MyWindowName, CV_GUI_NORMAL | WINDOW_AUTOSIZE);
}

void Screen2::DisplayImage() {
    imshow(MyWindowName, MyImage);
    waitKey(10);
    MyImage = MyBackground.clone();
}

void Screen2::DisplayBackground() {
    imshow(MyWindowName, MyBackground);
    waitKey(10);
}

void Screen2::UpdateSurrounding(double* scan) {
    Point obstacle[181];
    double radToDegree = RAD_TO_DEGREE;
    double rad;
    for (int i = 0; i < 181; i++) {
        rad = -i * radToDegree;

        if (scan[i] > MyLaserMaxRange) {
            scan[i] = MyLaserMaxRange;
        }
        obstacle[i].x = ((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x);
        obstacle[i].y = ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y);
        if (scan[i] < 500) {
            line(MyImage, MyLaserPosition, obstacle[i], Scalar(0, 0, 255));
        }
//        MyImage.at<Vec3b>(obstacle[i]) = colorBlack;

        if (i > 0) {
            if (i < 45 || i > 135) {
                if (norm(obstacle[i - 1] - obstacle[i]) > 100 / MyScaleY) {
                    line(MyImage, obstacle[i - 1], obstacle[i], Scalar(0, 0, 255));
                    continue;
                }
            }
            line(MyImage, obstacle[i - 1], obstacle[i], Scalar(0, 0, 0));
        }. /opt/ros/lunar/setup.bash
cd catkin_ws
. devel/setup.bash
clion
    }
//    double angleStep = robot->getRotVel() * TIME_STEP * DEGREE_TO_RAD;
//    double lengthStep = robot->getVel() * TIME_STEP;
//    Point estimatePath;
//    double angle, length;
//    for (int i = 1; i < 100; i++) {
//        angle = angleStep * i;
//        length = lengthStep * i;
//        estimatePath = Point(static_cast<int>(sin(-angle) * length / MyScaleX + MyRobotPosition.x),
//                             static_cast<int>(-cos(angle) * length / MyScaleY + MyRobotPosition.y));
//        if (estimatePath.y > MyRobotPosition.y) break;
//        MyImage.at<Vec3b>(estimatePath) = colorBlue;
//    }
//    rectangle(MyImage, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
//              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 200, 200), FILLED);


}

int Screen2::SearchFreeSpace(double *scan, double distThres, int countThres) {
    int count = 0;
    int mid = -1;
    int leftMid = -1;
    double radToDegree = RAD_TO_DEGREE;
    double rad;
    int i;
    Point midPoint, freeSpacePoint;
    for (i = 0; i < 181; i++) {
        if (scan[i] > distThres) {
            if (count == 0) mid = -1;
            count++;
            rad = -i * radToDegree;
            freeSpacePoint.x = ((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x);
            freeSpacePoint.y = ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y);
            if (scan[i] > distThres) {
                line(MyImage, MyLaserPosition, freeSpacePoint, Scalar(0, 255, 255));
            }
        }
        if (scan[i] <= distThres) {
            if (count > countThres) {
                mid = i - count / 2;
                rad = -mid * radToDegree;
                if (scan[i] > MyLaserMaxRange) {
                    scan[i] = MyLaserMaxRange;
                }
                midPoint = Point(((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x),
                                 ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y));
                line(MyImage, MyLaserPosition, midPoint, Scalar(0, 255, 0));
                if (mid > leftMid) {
                    leftMid = mid;
                }
            }
            count = 0;
        }
    }

    if (count > countThres && leftMid == -1) {
        leftMid = i - count / 2;
        if (scan[i] > MyLaserMaxRange) {
            scan[i] = MyLaserMaxRange;
        }
    }
    if (leftMid >= 0) {
        rad = -leftMid * radToDegree;
        midPoint = Point(((int) (scan[leftMid] * cos(rad) / MyScaleX) + MyLaserPosition.x),
                         ((int) (scan[leftMid] * sin(rad) / MyScaleY) + MyLaserPosition.y));
        line(MyImage, MyLaserPosition, midPoint, Scalar(0, 140, 250));
    }
    return leftMid;
}

int main(int argc, char** argv){
    

    return (0);
}