#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace zbar;

//g++ main.cpp /usr/local/include/ /usr/local/lib/ -lopencv_highgui.2.4.8 -lopencv_core.2.4.8

class QRDetector {
private:
    VideoCapture cap;
    ImageScanner scanner;
public:
    QRDetector(int videoSrc) {
        cap = VideoCapture(videoSrc);
        if (!cap.isOpened()) { // if not success, exit program
            cout << "Cannot open the video cam" << endl;
            exit(EXIT_FAILURE);
        }
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
        namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    }

    string readQR() {
        Mat frame, grey;
        string result;
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess) {
            cout << "Cannot read a frame from video stream" << endl;
            exit(EXIT_FAILURE);
        }
        cvtColor(frame, grey, CV_BGR2GRAY);
        int width = frame.cols;
        int height = frame.rows;
        uchar *raw = (uchar *) grey.data;
        // wrap image data
        Image image(width, height, "Y800", raw, width * height);
        // scan the image for barcodes
        int n = scanner.scan(image);
        // extract results
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            vector<Point> vp;
            result = symbol->get_data();
            int n = symbol->get_location_size();
            for (int i = 0; i < n; i++) {
                vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
            }
            RotatedRect r = minAreaRect(vp);
            Point2f pts[4];
            r.points(pts);
            for (int i = 0; i < 4; i++) {
                line(frame, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0), 3);
            }
        }
        imshow("MyVideo", frame); //show the frame in "MyVideo" window
        waitKey(1);
        return result;
    }
};
