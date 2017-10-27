//
// Created by chaoz on 22/10/17.
//

#include "StateDisplay.h"


StateDisplay::StateDisplay() {
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
    MyWindowName = "ChaoZ";
    MyLaserOffset = 125;
    MyLaserMaxRange = 8000;
    int scaleFactor = 13;
    MyScaleX = MyScaleY = scaleFactor;
    MyFrontLength = 313 / scaleFactor;
    MyHalfWidth = 253 / scaleFactor;
    MyRobotPosition = Point((int) (MyLaserMaxRange / MyScaleX),
                            (int) ceil((MyLaserMaxRange + MyLaserOffset) / MyScaleY));
    MyLaserPosition = MyRobotPosition;
    MyLaserPosition.y -= MyLaserOffset / MyScaleY;
    MyBackground = Mat(MyRobotPosition.y + 1,
                       (int) (2 * MyLaserMaxRange / MyScaleX) + 1, CV_8UC3, Scalar(255, 255, 255));
    rectangle(MyBackground, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 69, 255), -1);
    circle(MyBackground, MyLaserPosition, MyLaserPosition.x, Scalar(0, 255, 0), 1);
    rectangle(MyBackground,Point(1,1),Point(11,11),Scalar(0,255,0),-1);
    rectangle(MyBackground,Point(1,13),Point(11,23),Scalar(0,255,255),-1);
    rectangle(MyBackground,Point(1,25),Point(11,35),Scalar(255,0,0),-1);
    rectangle(MyBackground,Point(1,37),Point(11,47),Scalar(0,69,255),-1);
    rectangle(MyBackground,Point(1,49),Point(11,59),Scalar(0,0,0),-1);
    rectangle(MyBackground,Point(1,61),Point(11,71),Scalar(255,0,255),-1);
    putText(MyBackground, "8m Range", Point(11, 12), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "Open Space", Point(11, 24), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "Estimated Path", Point(11, 36), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "Pioneer 3at", Point(11, 48), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    putText(MyBackground, "Obstacle" , Point(11,60),FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
    putText(MyBackground, "Following Path", Point(11,72),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));

    putText(MyBackground, "mm/s", Point(MyBackground.cols-51,12),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground, "deg/s", Point(MyBackground.cols-51,24),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground, "deg", Point(MyBackground.cols-51,36),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground, "TransVel:", Point(MyBackground.cols-191,12),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground, "RotVel:", Point(MyBackground.cols-191,24),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground, "TurnAng:", Point(MyBackground.cols-191,36),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));

    MyBackground.copyTo(MyImage);
    MySize.width = (int) (2 * MyLaserMaxRange / MyScaleX);
    MySize.height = (int) (MyLaserMaxRange / MyScaleY);
    namedWindow(MyWindowName, CV_GUI_NORMAL | WINDOW_AUTOSIZE);
}

void StateDisplay::DisplayImage() {
    rectangle(MyImage, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 0, 255), -1);
    imshow(MyWindowName, MyImage);
    waitKey(10);
}

void StateDisplay::DisplayBackground() {
    imshow(MyWindowName, MyBackground);
    waitKey(10);
}

void StateDisplay::UpdateSurrounding(double *scan) {
    Point obstacle[181];
    double radToDegree = DEGREE_TO_RAD;
    double rad = 0.0;
    for (int i = 0; i < 181; i++) {
        rad = -i * radToDegree;
        if (scan[i] > MyLaserMaxRange) {
            scan[i] = MyLaserMaxRange;
        }
        obstacle[i].x = ((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x);
        obstacle[i].y = ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y);


        if (scan[i] < 350) {
            line(MyImage, MyLaserPosition, obstacle[i], Scalar(0, 0, 255));
        }
//        MyImage.at<Vec3b>(obstacle[i]) = colorBlack;

        if (i > 0) {
//            if (i < 45 || i > 135) {
//                if (norm(obstacle[i - 1] - obstacle[i]) > 100 / MyScaleY) {
//                    line(MyImage, obstacle[i - 1], obstacle[i], Scalar(0, 0, 255));
//                    continue;
//                }
//            }
            line(MyImage, obstacle[i - 1], obstacle[i], Scalar(0, 0, 0),2);
        }
    }
}

int StateDisplay::SearchFreeSpace(double *scan, double distThres, int countThres, double angle) {

    int count = 0;
    int mid = -1;
    int followMid = -1;
    double radToDegree = DEGREE_TO_RAD;
    double rad;
    double sumRange = 0.0;
    double halfSumRange = 0.0;
    int i;
    Point midPoint, freeSpacePoint;
    for (i = 0; i < 181; i++) {
        if (scan[i] > distThres) {
            if (count == 0) mid = -1;
            count++;
            sumRange += scan[i];
            rad = -i * radToDegree;
            freeSpacePoint.x = ((int) (scan[i] * cos(rad) / MyScaleX) + MyLaserPosition.x);
            freeSpacePoint.y = ((int) (scan[i] * sin(rad) / MyScaleY) + MyLaserPosition.y);
            if (scan[i] > distThres) {
                line(MyImage, MyLaserPosition, freeSpacePoint, Scalar(0, 255, 255),2);
            }
        }
        if (scan[i] <= distThres) {
            if (count > countThres) {
                halfSumRange = 0;
//                for (int j = i - count; j < i; j++) {
//                    halfSumRange += scan[j];
//                    if (halfSumRange > sumRange / 2) {
//                        mid = j;
//                        break;
//                    }
//                }
                mid = i - count / 2;
                rad = -mid * radToDegree;
                if (scan[i] > MyLaserMaxRange) {
                    scan[i] = MyLaserMaxRange;
                }
                midPoint = Point(((int) (scan[mid] * cos(rad) / MyScaleX) + MyLaserPosition.x),
                                 ((int) (scan[mid] * sin(rad) / MyScaleY) + MyLaserPosition.y));
                if (followMid == -1) {
                    followMid = mid;
                }
                if (abs(mid - angle) < abs(followMid - angle)) {
                    followMid = mid;
                }
            }
            count = 0;
            sumRange = 0;

        }
    }

    if (count > countThres && followMid == -1) {
        followMid = i - count / 2;
        if (scan[followMid] > MyLaserMaxRange) {
            scan[followMid] = MyLaserMaxRange;
        }
    }
    if (followMid >= 0) {
        rad = -followMid * radToDegree;
        midPoint = Point(((int) (scan[followMid] * cos(rad) / MyScaleX) + MyLaserPosition.x),
                         ((int) (scan[followMid] * sin(rad) / MyScaleY) + MyLaserPosition.y));
        line(MyImage, MyLaserPosition, midPoint, Scalar(255, 0, 255),2);
    }

    stringstream strstream;
    if(followMid!=-1){
        if(followMid<90&&followMid>=10) strstream<<"   "<<(90-followMid);
        if(followMid<10) strstream<<"    "<<(90-followMid);
        if(followMid>=100) strstream<<"  -"<<(followMid-90);
        if(followMid<100&&followMid>90) strstream<<"   -"<<(followMid-90);
        if(followMid==90) strstream<<"    0";
    }
    if(followMid==-1) strstream<<" --- ";
    putText(MyImage,strstream.str(),Point(MyImage.cols-101,36),FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    return followMid;
}

void StateDisplay::AddWayPoint(double dist, double ang, Scalar color) {
    Point waypoint;
    double rad;
    rad = ang * DEGREE_TO_RAD;
    if (dist > MyLaserMaxRange) {
        dist = MyLaserMaxRange - 15;
    }
    waypoint.x = ((int) (dist * -sin(rad) / MyScaleX) + MyRobotPosition.x);
    waypoint.y = ((int) (dist * -cos(rad) / MyScaleY) + MyRobotPosition.y);
    if (waypoint.y > MyRobotPosition.y) {
        waypoint.y = MyRobotPosition.y;
    }
    circle(MyImage, waypoint, 2, color, -1);
}

void StateDisplay::Clear() {
    MyImage = MyBackground.clone();
}

void StateDisplay::SaveImage(string name) {
    imwrite(name, MyImage);
}

void StateDisplay::AddLaserPoint(double dist, double ang, Scalar color) {
    Point waypoint;
    double rad;
    rad = ang * DEGREE_TO_RAD;
    if (dist > MyLaserMaxRange) {
        dist = MyLaserMaxRange - 15;
    }
    waypoint.x = ((int) (dist * -sin(rad) / MyScaleX) + MyLaserPosition.x);
    waypoint.y = ((int) (dist * -cos(rad) / MyScaleY) + MyLaserPosition.y);
    if (waypoint.y > MyLaserPosition.y) {
        waypoint.y = MyLaserPosition.y;
    }
    circle(MyImage, waypoint, 2, color, -1);
}

void StateDisplay::MotionEstimate(double linearVel, double angularVel) {
    double angleStep = angularVel * TIME_STEP;
    double lengthStep = linearVel * TIME_STEP;
    Point lastEstimate;
    Point estimatePath;
    double angle, length;
    lastEstimate = MyRobotPosition;
    for (int i = 1; i < 100; i++) {
        angle = angleStep * i;
        length = lengthStep * i;
        estimatePath = Point(static_cast<int>(sin(-angle) * length / MyScaleX + lastEstimate.x),
                             static_cast<int>(-cos(angle) * length / MyScaleY + lastEstimate.y));
        if (estimatePath.y > MyRobotPosition.y) break;
        circle(MyImage,estimatePath,2,Scalar(255,0,0),-1);
        lastEstimate = estimatePath;
    }
    stringstream strstream;
    strstream<<(int)(linearVel*1000);
    putText(MyImage,strstream.str(),Point(MyImage.cols-101,12),FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
    strstream<<(int)(angularVel*180/M_PI);
    putText(MyImage,strstream.str(),Point(MyImage.cols-101,24),FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));


}
