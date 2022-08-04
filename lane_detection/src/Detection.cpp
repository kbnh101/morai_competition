#include "./Detection.h"

Detection::Detection()
{
}

Detection::Detection(ros::NodeHandle node)
{
    subImg = node.subscribe("/image_jpeg/compressed", 10, &Detection::subImgCallback, this);
    state_sub = node.subscribe("state",10,&Detection::state_callback,this);

    cmd_pub = node.advertise<geometry_msgs::Twist>("lane",10);
}


void Detection::subImgCallback(const sensor_msgs::CompressedImage &subImgMsgs)
{
    // image 640 X 480
    check = true;
    mImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
    mImage = mImagePtr->image;
    auto start = std::chrono::high_resolution_clock::now();
    if(state.data == "lane")
    {
        run();
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time : " << duration.count() << " ms" << std::endl;
}

void Detection::state_callback(const std_msgs::String &msg)
{
    state = msg;
}

void Detection::angle(int x, int y)
{
    lane_cmd.linear.x = 25;
    double rotated_x, rotated_y = 0;

    rotated_x = x;
    rotated_y = -y + 480;

    double back_x = 320;
    double back_y = 480;

    double tx = back_x - x;
    double ty = back_y - y;

    double dis = 0;

    double lfd = 1;
    double max_lfd = 15;
    double min_lfd = 10;

    lfd = lane_cmd.linear.x/5.0;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    double dx,dy = 0;
    dx = x - 320;
    dy = y;

    if(dx > 0)
    {
        dis = sqrt(pow(dx,2) + pow(dy,2));
        if(dis>=lfd)
        {
            is_look_foward_point = true;
        }
        else
        {
            is_look_foward_point = false;
        }
    }

    double theta = atan2(dx,dy);
    double steering = 0;
    if(is_look_foward_point == true)
    {
        double eta = atan2((2*3.0*sin(theta)),lfd);
        steering = eta;
    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    lane_cmd.angular.z = steering;
    cmd_pub.publish(lane_cmd);
}

void Detection::run()
{
    int row = 640;
    int cols = 480;

    //out_dst=shadow_processing(mImage);
    bdst=bird_view(mImage);
    result=filter_colors(bdst);

    pixel_point right_pt;
    pixel_point left_pt;

    for(int i = cols; i>0; i--)
    {
        for(int j = row/2; j<row; j++)
        {
            if(result.at<uchar>(i,j) == 255)
            {
                right_pt.x = j;
                right_pt.y = i;
                this->right_point.push_back(right_pt);
                break;
            }
        }
    }

    for(int i = cols; i>0; i--)
    {
        for(int j = row/2; j>0; j--)
        {
            if(result.at<uchar>(i,j) == 255)
            {
                left_pt.x = j;
                left_pt.y = i;
                this->left_point.push_back(left_pt);
                break;
            }
        }
    }

    pixel_point avg_right_pt, avg_left_pt;
    double right_xsum, right_ysum = 0;
    double left_xsum, left_ysum = 0;

    for(int i = 0; i<right_point.size(); i++)
    {
        right_xsum += right_point.at(i).x;
        right_ysum += right_point.at(i).y;
    }

    avg_right_pt.x = right_xsum/right_point.size();
    avg_right_pt.y = right_ysum/right_point.size();

    for(int i = 0; i<left_point.size(); i++)
    {
        left_xsum += left_point.at(i).x;
        left_ysum += left_point.at(i).y;
    }

    avg_left_pt.x = left_xsum/left_point.size();
    avg_left_pt.y = left_ysum/left_point.size();

    int x,y = 0;

    x = (avg_left_pt.x + avg_right_pt.x)/2;
    y = (avg_left_pt.y + avg_right_pt.y)/2;

    result.at<uchar>(y,x) = 255;
    cv::circle(result, Point(x,y), 50, Scalar(255,255,255), 1,8,0);
    angle(x,y);

    cv::imshow("result", result);
    cv::waitKey(1);

    this->left_point.clear();
    this->right_point.clear();
}

cv::Mat Detection::shadow_processing(cv::Mat src) {

    Point2f src_point[4];

    src_point[0] = Point(0, 280);

    src_point[1] = Point(0, 380);

    src_point[2] = Point(640, 280);

    src_point[3] = Point(640, 380);

    Point2f dst_point[4];

    dst_point[0] = Point(0, 0);

    dst_point[1] = Point(0, src.rows);

    dst_point[2] = Point(src.cols, 0);

    dst_point[3] = Point(src.cols, src.rows);

    Mat h = getPerspectiveTransform(src_point, dst_point);
    Mat out;

    warpPerspective(src, out, h, src.size());

    for (int y = 0; y<out.rows; y++) {

        for (int x = 0; x < out.cols; x++) {

            for (float c = 0; c < 3; c++) {

                if (out.at<Vec3b>(y, x)[c] > 0 && out.at<Vec3b>(y, x)[c] < 58) {

                    for (int j = 0; j < y; j++)

                        out.at<Vec3b>(j, x)[c] = 100;

                }

            }

        }

    }

Point pt[1][4];
pt[0][0] = Point(0,0);
pt[0][1]=Point(190,0);
pt[0][2]=Point(80,480);
pt[0][3]=Point(0,480);
const Point * ppt[1] = {pt[0]};
int npt[]={4};
fillPoly(out,ppt,npt,1,Scalar(0,0,0),8);
    Mat roi(out, Rect(Point(0, 0), Point(90, 480)));

    roi = Scalar(0, 0, 0);
    return out;

}



Mat Detection::filter_colors(Mat src) {

    Mat bgr_src, hsv_src;

    src.copyTo(bgr_src);

    Mat white_mask, white_image;

    Mat yellow_mask, yellow_image;



    Scalar lower_white = Scalar(12, 20, 210);

    Scalar upper_white = Scalar(40, 55, 255);

    Scalar lower_yellow = Scalar(10, 100, 100);

    Scalar upper_yellow = Scalar(40, 255, 255);

    cvtColor(bgr_src, hsv_src, COLOR_BGR2HSV);

    inRange(hsv_src, lower_white, upper_white, white_mask);





    inRange(hsv_src, lower_yellow, upper_yellow, yellow_mask);



    bitwise_or(white_mask, yellow_mask, img_combine);
    return img_combine;

}

Mat Detection::bird_view(Mat &src){
    Point2f src_vertices[4];
//    src_vertices[0] = Point(163,176);
//    src_vertices[1] = Point(475,176);
//    src_vertices[2] = Point(570,468);
//    src_vertices[3] = Point(125,468);
    src_vertices[0] = Point(160,40);
    src_vertices[1] = Point(480,40);
    src_vertices[2] = Point(640,480);
    src_vertices[3] = Point(0,480);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0,0);
    dst_vertices[1] = Point(640,0);
    dst_vertices[2] = Point(640,480);
    dst_vertices[3] = Point(0,480);

    Mat M = getPerspectiveTransform(src_vertices,dst_vertices);
    Mat dst(480,640, CV_8UC3);

    warpPerspective(src,dst,M,dst.size(),INTER_LINEAR,BORDER_CONSTANT);
    Mat output;
    Mat kernel = (Mat_<int>(3,3)<<1,1,1,1,1,1,1,1,1);
    morphologyEx(dst,output,MORPH_OPEN,kernel);
    return output;
}
