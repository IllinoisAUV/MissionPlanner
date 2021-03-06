#include "vision.h"
#include <sstream>

//yellow:
//    inRange(src_hsv, Scalar(22,100,100), Scalar(38,255,255), lower_bound_image);
//    inRange(src_hsv, Scalar(22,100,100), Scalar(38,255,255), upper_bound_image);

//red/orange:
//    inRange(src_hsv, Scalar(0,0,179), Scalar(255,76,255), lower_bound_image);
//    inRange(src_hsv, Scalar(upper_bound1,100,100), Scalar(upper_bound2,255,255), upper_bound_image);


#define IMAGE_DEST ("/home/ubuntu/catkin_ws/image/")
#define WRITE_IMAGES (true)

Vision::Vision() {
    done_vgate = false;
    done_buoy = false;
    done_wire = false;
    index_ = 0;

    /* namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display. */
    /* namedWindow( "Contours", CV_WINDOW_AUTOSIZE ); */
    /* cout << "Opening video file" << endl; */
    if(!cap_.open(0)) {
        cout << "Cannot open video file" << endl;
        throw "Cannot open video file";
    }
    cap_.set(CV_CAP_PROP_BUFFERSIZE, 1);
    cap_.set(CV_CAP_PROP_FPS, 5);
}

targets_data Vision::findTargets(){
    targets_data ret;
    vector<double> r = detect_buoy(this->src);
    ret.buoy_angle = r[0];
    ret.buoy_radius = r[1];
    ret.buoy_area = r[2];
    /* ret.buoy_angle = -1; */
    /* ret.buoy_radius = -1; */
    /* ret.buoy_area = -1; */

    /* Mat src_colored = color(this->src); */
    /* r = outputWireAngle(src_colored); */
    /* ret.wire_angle = r[0]; */
    /* ret.wire_radius = r[1]; */
    /* ret.wire_area = r[2]; */
    ret.wire_angle = -1;
    ret.wire_radius = -1;
    ret.wire_area = -1;

    //TODO gate
    ret.vgate_angle = -1;
    ret.vgate_radius = -1;
    ret.vgate_area = -1;
    return ret;
}

void Vision::getImage() {
    if (!cap_.read(src)) {
        cout << "Failed to read" << endl;
    }
    if(!src.empty()) {
        std::stringstream dest;
#if WRITE_IMAGES
        dest << IMAGE_DEST << index_ << ".jpg";
        imwrite(dest.str(), src);
#endif
        resize(src, src, Size(640, 480));
        index_++;
        /* imshow("Raw Image", src); */
        waitKey(25);
    }
}
vector<double> Vision::detect_buoy(Mat src){
    Mat src_hsv, src_colored;
	medianBlur(src, src, 9);

    /* Mat src_ycrcb, result_ycrcb; */
    /* cvtColor(src, src_ycrcb, CV_BGR2YCrCb); */
    /* vector<Mat> channels; */
    /* split(src_ycrcb, channels); */
    /* equalizeHist(channels[0], channels[0]); */
    /* merge(channels,result_ycrcb); */
    /* cvtColor(result_ycrcb,src,CV_YCrCb2BGR); */

    cvtColor(src, src_hsv, COLOR_BGR2HSV);
    /* imshow("src", src); */
    int lower_bound1, lower_bound2;
    int upper_bound1, upper_bound2;
    string colorToDetect;
    Mat lower_bound_image, upper_bound_image;
    /* lower_bound1 = 1; */
    /* lower_bound2 = 40; */
    /* upper_bound1 = 1; */
    /* upper_bound2 = 40; */

    //filtering for color
    // yellow
    inRange(src_hsv, Scalar(10,100,100), Scalar(50,255,255), lower_bound_image);
    inRange(src_hsv, Scalar(10,100,100), Scalar(50,255,255), upper_bound_image);

    // red
    /* inRange(src_hsv, Scalar(0,0,179), Scalar(255,179,255), lower_bound_image); */
    /* inRange(src_hsv, Scalar(0,0,179), Scalar(255,179,255), upper_bound_image); */

    addWeighted(lower_bound_image, 1.0, upper_bound_image, 1.0, 0.0, src_colored);

	/* imshow("thresholded", src_colored); */


    //src colored is our filtered img
    RNG rng(12345);
    Mat output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //findContours function
    findContours(src_colored, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

    //approximate the contours to get bounding rectangles and circles
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> center(contours.size());
    vector<float> radius(contours.size());

    for(size_t i = 0; i < contours.size(); i++){
    	approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
    }

    //checking the radii, making sure greater than 100 for these purposes. -- can change
    vector<Point2f> candidates;
    double largest_radius = 0;
    double largest_radius2 = 0;
    double largest_idx = -1;
    double largest_idx2 = -1;
    for(size_t i = 0; i < contours.size(); i++){
        if(contourArea(contours[i]) > largest_radius && contourArea(contours[i]) > 0){
			largest_radius2 = largest_radius;
	    	largest_idx2 = largest_idx;
			largest_radius = contourArea(contours[i]);
	    	largest_idx = i;
		}
    }
    if(largest_idx2 != -1) {
        if(contours[largest_idx2][0].y < contours[largest_idx][0].y) {
            largest_idx = largest_idx2;
        }
    }

	//push the largest on
	vector<double> rvec;


	if(largest_idx != -1){
        candidates.push_back(center[largest_idx]);
	    outputBuoyAngle(src, candidates, rvec);
	}
	else{
	    /* cout << "Nothing detected, Angle to travel : -1" << endl; */
		rvec.push_back(-1);
		rvec.push_back(-1);
		rvec.push_back(-1);
		return rvec;
	}
	/* cout << "BUOY CONTOUR AREA: " <<  contourArea(contours[largest_idx]) << endl;; */

    Mat drawing = Mat::zeros(src_colored.size(), CV_8UC3);
   	Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
	drawContours(drawing, contours_poly, largest_idx, color, 1, 8, vector<Vec4i>(), 0, Point());
    rectangle(drawing, boundRect[largest_idx].tl(), boundRect[largest_idx].br(), color, 2, 8, 0);

	/* imshow("buoy contour", drawing); */



	//return the vector
        cout << boundRect[largest_idx].area()<< endl;
	rvec.push_back(boundRect[largest_idx].area());
	return rvec;
}

void Vision::outputBuoyAngle(Mat &src, vector<Point2f> &candidates, vector<double> &rvec){
    /* cout << "outputBuoyAngle" << endl; */
    //using the colored image
    //we have a point, which is the center of a cirlce, should only have 1.
    int midX = src.size().width / 2;
    int midY = src.size().height / 2;
    int x, y, tx, ty, targetx = -1, targety = -1;
    int flag = 0;
    /* cout << "SIZE OF CANDIDATES VECTOR: " << candidates.size() << endl; */
    for(size_t i = 0; i < candidates.size(); i++){
        if(flag != 0){
            break;
        }
        x = candidates[i].x;
        y = candidates[i].y;
        for(tx = -5; tx < 6; tx++){
            if(flag != 0)	{ break; }
            for(ty = -5; ty < 6; ty++){
                int correctColor = src.at<uchar>(y + ty, x + tx);
                if(correctColor != 0){
                    flag = 1;
                    targetx = x;
                    targety = y;
                    break;
                }
            }
        }
    }

    //now we know what point  we are going to travel to, so we want to calculate the angle

    //triangle
    //	    adj
    //       --------
    //	  \	|
    //	   \	|
    //	    \	|     opp
    //	     \	|
    //	      \	|
    //	       \|

    //	cout << "target x = " << targetx << " target y = " << targety << endl;
    //	cout << "midx = " << midX << " midy = " << midY << endl;

    if(targetx == -1 && targety == -1){
        cout << "Nothing detected, Angle to travel : -1" << endl;
        vector<double> r;
        r.push_back(-1);
        r.push_back(-1);
    }

    double adj = abs(targetx - midX);
    double opp = abs(targety - midY);
    double angle = atan(opp / adj);

    int quadrant;
    if(targety < midY && targetx > midX){
        quadrant = 1;
        cout << "Angle to travel : " << angle << endl;
    }
    else if(targety < midY && targetx < midX){
        quadrant = 2;
        cout << "Angle to travel : " << 3.14159 - angle << endl;
    }
    else if(targety > midY && targetx < midX){
        quadrant = 3;
        cout << "Angle to travel : " << 3.14159 + angle << endl;
    }
    else{
        quadrant = 4;
        cout << "Angle to travel : " << -angle << endl;
    }

    cout << __LINE__ << endl;
    rvec.push_back(angle);
    cout << __LINE__ << endl;
    rvec.push_back(sqrt((opp*opp) + (adj*adj)));
    cout << __LINE__ << endl;
}


Mat Vision::color(Mat src){
    Mat src_hsv, src_colored;
    cout << "color" << endl;
    cvtColor(src, src_hsv, COLOR_BGR2HSV);

    cout << "Coloring image" << endl;
    int lower_bound1, lower_bound2;
    int upper_bound1, upper_bound2;
    string colorToDetect;
    Mat lower_bound_image, upper_bound_image;
    lower_bound1 = 18;
    lower_bound2 = 60;
    upper_bound1 = 18;
    upper_bound2 = 60;

    inRange(src_hsv, Scalar(lower_bound1,50,70), Scalar(lower_bound2,255,255), lower_bound_image);
    inRange(src_hsv, Scalar(upper_bound1,50,70), Scalar(upper_bound2,255,255), upper_bound_image);

    addWeighted(lower_bound_image, 1.0, upper_bound_image, 1.0, 0.0, src_colored);
    dilate(src_hsv, src_hsv, getStructuringElement(MORPH_ELLIPSE, Size(31,31)));
    return src_colored;
}

vector<double> Vision::outputWireAngle(Mat &src_colored){
    cout << "outputWireAngle" << endl;
    if(!src.empty()) {
        vector<double> res;
        res.push_back(-1);
        res.push_back(-1);
        res.push_back(-1);
        return res;
    }
    //use contour area to find the biggest contour
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    int thresh = 100;
    int max_thresh = 255;

    /// Detect edges using Threshold
    threshold( src_colored, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    if(contours.size() < 3){
        vector<double> r;
        r.push_back(-1);
        r.push_back(-1);
        return r;
    }

    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );

    //map the areas with the indices
    vector<double> maxContours;
    map<double, int> contourMap;
    for(int i = 0; i < contours.size(); i++){
        contourMap[contourArea(contours[i])] = i;
        maxContours.push_back(contourArea(contours[i]));
    }

    sort(maxContours.begin(), maxContours.end());
    //get the 3 largest areas
    //make array of largest points
    reverse(maxContours.begin(), maxContours.end());

    double big1 = maxContours[0];
    double big2 = maxContours[1];
    double big3 = maxContours[2];

    cout << "first = " << big1 << "	second = " << big2 << "		third = " << big3 << endl;

    //get the 3 largest indices
    int idx1 = contourMap[big1];
    int idx2 = contourMap[big2];
    int idx3 = contourMap[big3];
    int minSizeContour = 50;
    int sizeOfNewContours = 0;
    if(big1 > minSizeContour){
        sizeOfNewContours++;
    }
    if(big2 > minSizeContour){
        sizeOfNewContours++;
    }
    if(big3 > minSizeContour){
        sizeOfNewContours++;
    }

    vector<vector<Point> > newContours;
    newContours.resize(sizeOfNewContours);

    //	cout << contours[idx1] << endl << contours[idx2] << endl << contours[idx3] << endl;
    //	for(int j =0; j < newContours.size(); j++){
    if(big1 > minSizeContour){
        for(int i = 0; i < contours[idx1].size(); i++){
            newContours[0].push_back(contours[idx1][i]);
        }
    }
    if(big2 > minSizeContour){
        for(int i = 0; i < contours[idx2].size(); i++){
            newContours[1].push_back(contours[idx2][i]);
        }
    }
    if(big3 > minSizeContour){
        for(int i = 0; i < contours[idx3].size(); i++){
            newContours[2].push_back(contours[idx3][i]);
        }
    }
    //	}

    for( int i = 0; i < newContours.size(); i++ )
    { minRect[i] = minAreaRect( Mat(newContours[i]) );
        if( newContours[i].size() > 5 )
        { minEllipse[i] = fitEllipse( Mat(newContours[i]) ); }
    }

    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< newContours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        // contour
        drawContours( drawing, newContours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        // ellipse
        //ellipse( drawing, minEllipse[i], color, 2, 8 );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    // Show in a window
    /* imshow( "Contours", drawing ); */
    waitKey(25);


    //need to do this*****
    vector<double> r;
    r.push_back(-1);
    r.push_back(-1);
    r.push_back(-1);
    return r;
}

