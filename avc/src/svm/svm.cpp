#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rr_platform/transform_image.h>

using namespace std;
using namespace cv;
using namespace ros;
using namespace cv::ml;

using uchar = unsigned char;

//img size: 480 x 640 for camera

Publisher img_pub;

void renderClassification(const Mat &labels, Mat &render) {
    render = labels.reshape(1, 20).clone();
    render *= 255;
    resize(render, render, Size(1000, 1000));
}

vector<float> avgPixel(const Mat &img, const int &r, const int &c) {
    float avgB = 0;
    float avgG = 0;
    float avgR = 0;
    for (int i = r; i < r + 50; i++) {
        for (int j = c; j < c + 50; j++) {
            avgB += img.at<Vec3b>(i, j).val[0];
            avgG += img.at<Vec3b>(i, j).val[1];
            avgR += img.at<Vec3b>(i, j).val[2];
        }
    }
    avgB /= 2500;
    avgG /= 2500;
    avgR /= 2500;
    return {avgB, avgG, avgR};
}

void buildData(Mat &frame, Mat &data, Mat &dbg_img) {
    Size size(1000, 1000);
    resize(frame, frame, size, 0, 0, INTER_AREA);
    dbg_img = frame.clone();
    cvtColor(dbg, dbg, COLOR_BGR2HSV);
    float *dataRow = data.ptr<float>(0);
    for (int r = 0; r < size.height; r+= 50) {
        for (int c = 0; c < size.height; c+= 50) {
            vector<float> avg = avgPixel(dbg_img, r, c);
            dataRow[0] = avg[0];
            dataRow[1] = avg[1];
            dataRow += data.step1();
        }
    } 
} 

void classifyImage(const sensor_msgs::ImageConstPtr& msg) {
    Ptr<SVM> svm = SVM::load("svm_trained.xml");
    cv_bridge::CvImagePtr cv_ptr;
    Mat frame;
    Mat labels;
    Mat output;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV-Bridge error %s", e.what());
        return
    }
    frame = cv_ptr->image;
    Mat data(400, 2, CV_32F);
    Mat dbg_img;
    buildData(frame, data, dbg_img);
    svm->predict(data, labels);
    renderClassification(labels, output);
    sensor_msgs::Image outmsg;
    cv_ptr->image = output;
    cv_ptr->encoding = "bgr8";
    cv_ptr->toImageMsg(outmsg);
    img_pub.publish(outmsg);
}

int main(int argc, char** argv) {

    init(argc, argv, "color_svm_avc");

    NodeHandle nh;

    img_pub = nh.advertise<sensor_msgs::Image>("/colors_img", 1);
    auto img_sub = nh.subscribe("/camera/image_rect", 1, classifyImage);

    spin();

    return 0;
}
