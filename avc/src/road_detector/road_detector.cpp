#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

ros::Publisher pub;


/*
	This Histogram Road Detection Algorithmn is based on:
	http://www.morethantechnical.com/2013/03/05/skin-detection-with-probability-maps-and-elliptical-boundaries-opencv-wcode/
	http://www.cse.unsw.edu.au/~icml2002/workshops/MLCV02/MLCV02-Morales.pdf
	http://www.sciencedirect.com/science/article/pii/S0031320306002767

	Be careful, a bad bootstrap will cause it to converge around non road.
*/



//VAR Declaration
cv::Mat road_histogram_hue;
cv::Mat nonRoad_histogram_hue;
cv::Mat shadows_histogram_hue;
cv::Mat nonShadows_histogram_hue;
cv::Mat road_mask;
cv::Mat shadows_mask;
cv::Mat compiled_mask;

cv::Mat road_histogram_hue_normalized(361,1,5); //%
cv::Mat nonRoad_histogram_hue_normalized(361,1,5); //%
cv::Mat shadows_histogram_hue_normalized(361,1,5); //%
cv::Mat nonShadows_histogram_hue_normalized(361,1,5); //%

const int hist_bins_hue = 361; //0-360 +1 exclusive
const int hist_bins_saturation = 101; //0-100 +1 exclusive

bool firstRun = true;



//###########################################################
cv::Mat calculateHistogram(cv::Mat image, cv::Mat histogram, int bins){
	bool uniform = true; bool accumulate = false;
	int histSize = bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360
	const float* histRange = { range };
	int channels [] = {0};

	cv::calcHist(&image,1,channels,cv::Mat(),histogram, 1 ,&histSize,&histRange,uniform,accumulate);
	return histogram;
}

cv::Mat calculateHistogramMask(cv::Mat image, cv::Mat histogram, cv::Mat mask, int bins){
	bool uniform = true; bool accumulate = false;//true;//false;
	int histSize = bins; //Establish number of BINS

	float range[] = { 0, bins } ; //the upper boundary is exclusive. HUE from 0 - 360.
	const float* histRange = { range };
	int channels [] = {0};

	cv::calcHist(&image,1,channels,mask,histogram, 1, &histSize,&histRange,uniform,accumulate);
	return histogram;
}



cv::Mat bootstrap(cv::Mat image_hue){
	float threshold = 0.16;//TODO: Play with this number 0-1

  const int rectangle_x = 670; //TODO: set rectangle more in the middle? or find a better calibration point.
	const int rectangle_y = 760;
	const int rectangle_width = 580;
	const int rectangle_height = 300;

	cv::Mat mask(image_hue.rows, image_hue.cols, CV_8UC1); //mask. Same width/height of input image. Greyscale.
	cv::Mat histogram_hue;

	cv::Mat image_rectangle_hue = image_hue(cv::Rect(rectangle_x,rectangle_y,rectangle_width, rectangle_height));

	histogram_hue = calculateHistogram(image_rectangle_hue,road_histogram_hue,hist_bins_hue);

	//Normalize (sums to 1). Basically gives percentage of pixels of that color (0-1).
	int rectangle_pixels = (rectangle_width * rectangle_height);
	for (int ubin = 0; ubin < hist_bins_hue; ubin++) {
		if(histogram_hue.at<float>(ubin) > 0){
			histogram_hue.at<float>(ubin) /= rectangle_pixels;
		}else{
			histogram_hue.at<float>(ubin) = 0;
		}
	}


	//Mask making
	for(auto i = 0; i < image_hue.rows; i++){
		const unsigned char* row_hue = image_hue.ptr<unsigned char>(i); // HUE
		unsigned char* out_row = mask.ptr<unsigned char>(i); //output

		for(auto j = 0; j < image_hue.cols; j++){
			auto hue = row_hue[j]; //HUE of pixel
			auto histogram_hue_value = histogram_hue.at<float>(hue);

			if(histogram_hue_value >= threshold){
				out_row[j] = 255; //Road. White.
			}else{
				out_row[j] = 0; //nonRoad. Black.
			}

		}

	}

	return mask;
}

//##### #TODO DO NOT FORGET TO RUN road_detector_histogramTrainer.
cv::Mat bootstrapLoadFromStorage(cv::Mat image_hue, float threshold, string fileName, int isShadows){

	cv::Mat mask(image_hue.rows, image_hue.cols, CV_8UC1); //mask. Same width/height of input image. Greyscale.
	cv::Mat histogram_hue;

	//FILE LOADING HISTOGRAM
	// load file
	// per http://stackoverflow.com/questions/10277439/opencv-load-save-histogram-data
	cv::FileStorage fs(fileName + ".yml", cv::FileStorage::READ);
	if (!fs.isOpened()) {ROS_FATAL_STREAM("unable to open file storage!");}
	fs[fileName] >> histogram_hue;
	fs.release();

	//Mask making
	for(auto i = 0; i < image_hue.rows; i++){
		const unsigned char* row_hue = image_hue.ptr<unsigned char>(i); // HUE
		unsigned char* out_row = mask.ptr<unsigned char>(i); //output

		for(auto j = 0; j < image_hue.cols; j++){
			auto hue = row_hue[j]; //HUE of pixel
			auto histogram_hue_value = histogram_hue.at<float>(hue);

			if(histogram_hue_value >= threshold){
				out_row[j] = 255; //Road. White.
			}else{
				out_row[j] = 0; //nonRoad. Black.
			}

		}

	}
	if(isShadows == 0) {
		road_histogram_hue_normalized = histogram_hue;
	}
	if(isShadows == 1) {
		shadows_histogram_hue_normalized = histogram_hue;
	}

	return mask;
}

void trainOnlyOne(cv::Mat &hist_hue_road, cv::Mat mask){
	//Normalize (sums to 1). Basically gives percentage of pixels of that color (0-1).
	int road_pixels = cv::countNonZero(mask); int nonRoad_pixels = cv::countNonZero(~mask);
	for (int ubin = 0; ubin < hist_bins_hue; ubin++) {
		if(hist_hue_road.at<float>(ubin) > 0){
			hist_hue_road.at<float>(ubin) /= road_pixels;
		}
	}

}


void train(cv::Mat &hist_hue_road,cv::Mat &hist_hue_nonRoad, cv::Mat mask){
	//Normalize (sums to 1). Basically gives percentage of pixels of that color (0-1).
	int road_pixels = cv::countNonZero(mask); int nonRoad_pixels = cv::countNonZero(~mask);
	for (int ubin = 0; ubin < hist_bins_hue; ubin++) {
		if(hist_hue_road.at<float>(ubin) > 0){
			hist_hue_road.at<float>(ubin) /= road_pixels;
		}
		if(hist_hue_nonRoad.at<float>(ubin) > 0){
			hist_hue_nonRoad.at<float>(ubin) /= nonRoad_pixels;
		}
	}

}


cv::Mat predict(cv::Mat img_hue, cv::Mat &hist_hue_road, cv::Mat &hist_hue_nonRoad, float threshold){

	//create empty mask			ROS_FATAL_STREAM(road_histogram_hue_normalized.size()); //%
	cv::Mat mask(img_hue.rows, img_hue.cols, CV_8UC1);

	//Mask making based on threshold of road/nonRoad
	for(auto i = 0; i < img_hue.rows; i++){
		const unsigned char* row_hue = img_hue.ptr<unsigned char>(i); // HUE
		unsigned char* out_row = mask.ptr<unsigned char>(i); //output

		for(auto j = 0; j < img_hue.cols; j++){
			auto hue = row_hue[j]; //HUE of pixel
			auto hist_hue_road_val = hist_hue_road.at<float>(hue);
			auto hist_hue_nonRoad_val = hist_hue_nonRoad.at<float>(hue);

			if(hist_hue_nonRoad_val > 0){
				if((hist_hue_road_val / hist_hue_nonRoad_val) >= threshold){
					out_row[j] = 255; //Road. White.
				}else{
					out_row[j] = 0; //nonRoad. Black.
				}
			}else{
					out_row[j] = 0; //nonRoad. Black. //TODO:!!!! WHY THIS? why not check hist_hue_road_val>0 and set white??
			}

		}
	}

	return mask;
}



void img_callback(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv::Mat frame = cv_ptr->image;

		cv::Mat frame_hsv;
		cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV );
		vector<cv::Mat> frame_hsv_planes;

		cv::split(frame_hsv, frame_hsv_planes);

		if(firstRun){
			//Guess of road from stored histogram!
			road_mask = bootstrapLoadFromStorage(frame_hsv_planes[0],0.12,"road_histogram_hue_normalized",0);//#TODO: check
			shadows_mask = bootstrapLoadFromStorage(frame_hsv_planes[0],0.091,"shadows_histogram_hue_normalized",1);//#TODO: mess with threshold
			trainOnlyOne(road_histogram_hue_normalized, road_mask);
			trainOnlyOne(shadows_histogram_hue_normalized,shadows_mask);
			firstRun = false;
		}

		//cutout Horizon/sky #TODO: cut at actual horizon?
		rectangle(road_mask,cv::Point(0,0),cv::Point(road_mask.cols,road_mask.rows / 3), cv::Scalar(0,0,0),CV_FILLED);
		rectangle(shadows_mask,cv::Point(0,0),cv::Point(road_mask.cols,road_mask.rows / 3), cv::Scalar(0,0,0),CV_FILLED);

		road_histogram_hue = calculateHistogramMask(frame_hsv_planes[0],road_histogram_hue,road_mask,hist_bins_hue); //road
		nonRoad_histogram_hue = calculateHistogramMask(frame_hsv_planes[0],nonRoad_histogram_hue,~road_mask,hist_bins_hue); //nonRoad
		train(road_histogram_hue,nonRoad_histogram_hue,road_mask);

		shadows_histogram_hue = calculateHistogramMask(frame_hsv_planes[0],shadows_histogram_hue,shadows_mask,hist_bins_hue); //shadows
		nonShadows_histogram_hue = calculateHistogramMask(frame_hsv_planes[0],nonShadows_histogram_hue,~shadows_mask,hist_bins_hue); //nonShadows
		train(shadows_histogram_hue,nonShadows_histogram_hue,shadows_mask);

		//mix normalize maps
		road_histogram_hue_normalized = (road_histogram_hue_normalized * 0.95 + road_histogram_hue * 0.05); /// 2;  //% TODO should weight averages to give previous frames more weight
		nonRoad_histogram_hue_normalized = (nonRoad_histogram_hue_normalized * 0.95 + nonRoad_histogram_hue * 0.05);// / 2; //%

		shadows_histogram_hue_normalized = (shadows_histogram_hue_normalized * 0.95 + shadows_histogram_hue * 0.05);// / 2;  //% TODO should weight averages to give previous frames more weight
		nonShadows_histogram_hue_normalized = (nonShadows_histogram_hue_normalized * 0.95 + nonShadows_histogram_hue * 0.05);// / 2; //%

		const float road_threshold = 2.7;//2.0; //TODO: Play with this value
		const float shadows_threshold = 1.9;//3.3; //TODO: Play with this value
		road_mask = predict(frame_hsv_planes[0], road_histogram_hue_normalized, nonRoad_histogram_hue_normalized, road_threshold);
		shadows_mask = predict(frame_hsv_planes[0], shadows_histogram_hue_normalized, nonShadows_histogram_hue_normalized, shadows_threshold);
		//#TODO:train-predict loop?

		//cutout Horizon #TODO: cut at actual horizon? Also there is a better way to do this right?
		rectangle(road_mask,cv::Point(0,0),cv::Point(road_mask.cols,road_mask.rows / 3), cv::Scalar(0,0,0),CV_FILLED);
		rectangle(shadows_mask,cv::Point(0,0),cv::Point(road_mask.cols,road_mask.rows / 3), cv::Scalar(0,0,0),CV_FILLED);


		cv::imwrite("/home/brian/50Road1.png",road_mask);
		cv::imwrite("/home/brian/51Shadows1.png",shadows_mask);
/*
		auto kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4));
		cv::morphologyEx(shadows_mask,shadows_mask,cv::MORPH_CLOSE,kernel);
		cv::imwrite("/home/brian/52Shadows2_op.png",shadows_mask);
		cv::morphologyEx(road_mask,road_mask,cv::MORPH_CLOSE,kernel);
		cv::imwrite("/home/brian/51Road_op.png",road_mask);
*/
		cv::bitwise_or(road_mask, shadows_mask, compiled_mask);
		//morphologyEx Closing operation. #TODO:Keep trying filtering below

	//## may be better solution!! Works 8-28-17 just needs a large kernal the end to destroy last black spots
	cv::imwrite("/home/brian/10before.png",compiled_mask);
	auto kernel1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4));
	cv::morphologyEx(compiled_mask,compiled_mask,cv::MORPH_CLOSE,kernel1);
	cv::imwrite("/home/brian/11close.png",compiled_mask);
	auto kernel2 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(10,10));
	cv::morphologyEx(compiled_mask,compiled_mask,cv::MORPH_OPEN,kernel2);
	cv::imwrite("/home/brian/12open.png",compiled_mask);

	//Clear out last random black noise. Warning: Loss of detail
	auto kernel3 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(30,30)); //Larger num = no noise but loose exactness. Try 50.
	cv::morphologyEx(road_mask,road_mask,cv::MORPH_CLOSE,kernel3);
	cv::imwrite("/home/brian/13close.png",compiled_mask);


	//########May remove this or change it or just merge with above.
//	auto kernel4 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(60,60)); //Larger num = no noise but loose exactness. Try 50.
//	cv::morphologyEx(compiled_mask,compiled_mask,cv::MORPH_CLOSE,kernel4);
//	cv::imwrite("/home/brian/14close.png",compiled_mask);
	//###############################


		/* THE PLAN:
		bootstrap mask (guess from rectangle) (or guess from histogram from file)
		->
		calculate Histograms for road and non road;
		train those histograms (make them each add to 1)
		average noramalized histograms
		predict(create a new mask based on threshold compare of road and nonRoad)
		(repeat at -> x times)
		filter blur (something that checks if pixels around it are white, then makes it white or black based on that)
		*/

    //publish image
    sensor_msgs::Image outmsg;
    cv_ptr->image = compiled_mask;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);

    pub.publish(outmsg);
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "road_detector");

	ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::Image>("/road_detector", 1); //test publish of image
	auto img_sub = nh.subscribe("/camera/image_rect", 1, img_callback);

	ros::spin();
	return 0;

}
