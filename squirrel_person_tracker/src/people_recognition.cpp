#include <squirrel_person_tracker/people_recognition.h>

using namespace cv;
using namespace std;

ShirtDetector::ShirtDetector(const std::string& file_path, ros::NodeHandle& nh) :
        nh_(nh),gh() {
    std::string camera;
    if (!nh_.getParam("camera", camera)) {
      camera = "camera";
      ROS_INFO("camera is not set. Default value: camera, NH NS %s",nh_.getNamespace().c_str());
    }
    cascadeFileFace = file_path;
    image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(gh, camera + "/rgb/image_raw", 1));
    pc_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(gh, camera + "/depth/points", 1));
    sync.reset(new message_filters::Synchronizer<SyncPolicy>(10, *image_sub_, *pc_sub_));
    sync->registerCallback(boost::bind(&ShirtDetector::sync_cb, this, _1, _2));
    imagePub_ = nh_.advertise<sensor_msgs::Image>("shirtImage", 1);
    poseArrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("facePoints", 1);
    shirtPub_ = nh_.advertise<squirrel_person_tracker_msgs::ShirtMsg>("shirtInfo", 1);

}

/**
 Function to convert 2D pixel point to 3D point by extracting point
 from PointCloud2 corresponding to input pixel coordinate. This function
 can be used to get the X,Y,Z coordinates of a feature using an
 RGBD camera, e.g., Kinect.
 */
void ShirtDetector::pixelTo3DPoint(const sensor_msgs::PointCloud2& pCloud, const int u, const int v, geometry_msgs::Point &p) {
    // get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v * pCloud.row_step + u * pCloud.point_step;

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    // put data into the point p
    p.x = X;
    p.y = Y;
    p.z = Z;

}
void ShirtDetector::sync_cb(const sensor_msgs::ImageConstPtr& imgIn, const sensor_msgs::PointCloud2ConstPtr& pc) {

    std::vector<Rect> rectShirt;
    squirrel_person_tracker_msgs::ShirtMsg msg;
    geometry_msgs::PoseArray poseArrayMsg;
    msg.header.frame_id = poseArrayMsg.header.frame_id = "camera_rgb_optical_frame";
    msg.header.stamp = poseArrayMsg.header.stamp = ros::Time::now();
    geometry_msgs::Point gpoint;
    getImgForShirt(imgIn, rectShirt, msg);
    for (int i = 0; i < rectShirt.size(); ++i) {
        unsigned int px = rectShirt[i].x + 0.5 * rectShirt[i].width;
        unsigned int py = rectShirt[i].y + 0.5 * rectShirt[i].height;
        pixelTo3DPoint(*pc, px, py, gpoint);
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position = gpoint;
        poseArrayMsg.poses.push_back(pose);
        msg.pose.push_back(pose);
    }
    poseArrayPub_.publish(poseArrayMsg);
    shirtPub_.publish(msg);
    ROS_INFO("SYNC CB!!!!!!!!!!!!!!!!!!!!!");

}
void ShirtDetector::getImgForShirt(const sensor_msgs::ImageConstPtr& imgIn, std::vector<Rect>& rectFaces,
                                   squirrel_person_tracker_msgs::ShirtMsg& msg) {

    std::cout << "Loading Face HaarCascade in '" << cascadeFileFace << "'" << std::endl;
    if (!cascadeFace.load(cascadeFileFace)) {
        std::cerr << "ERROR: Couldn't load face detector classifier in '" << cascadeFileFace << "'\n";
        exit(1);
    }
    std::cout << "Done!" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(imgIn, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat &tmpImg = cv_ptr->image;
    IplImage* imageIn = new IplImage(tmpImg);
    std::cout << "(got a " << imageIn->width << "x" << imageIn->height << " color image)." << std::endl;
    IplImage* imageDisplay = cvCloneImage(imageIn);

    // First, search for all the frontal faces in the image
    Rect foundFace = Rect(0, 0, 0, 0); // Set init values if nothing was detected.

    double timeFaceDetectStart = (double)cvGetTickCount(); // Record the timing.
    cascadeFace.detectMultiScale(imageIn, rectFaces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

    msg.color.resize(rectFaces.size());
    msg.safety.resize(rectFaces.size());
    // Process each detected face
    std::cout << "Detecting shirt colors below the faces." << std::endl;
    for (int r = 0; r < rectFaces.size(); r++) {
        float initialConfidence = 1.0f;
        int bottom;
        Rect rectFace = rectFaces[r];
        drawRectangle(imageDisplay, rectFace, CV_RGB(255, 0, 0));

        // Create the shirt region, to be below the detected face and of similar size.
        float SHIRT_DY = 1.4f; // Distance from top of face to top of shirt region, based on detected face height.
        float SHIRT_SCALE_X = 0.6f; // Width of shirt region compared to the detected face
        float SHIRT_SCALE_Y = 0.6f; // Height of shirt region compared to the detected face
        Rect rectShirt;
        rectShirt.x = rectFace.x + (int)(0.5f * (1.0f - SHIRT_SCALE_X) * (float)rectFace.width);
        rectShirt.y = rectFace.y + (int)(SHIRT_DY * (float)rectFace.height)
                + (int)(0.5f * (1.0f - SHIRT_SCALE_Y) * (float)rectFace.height);
        rectShirt.width = (int)(SHIRT_SCALE_X * rectFace.width);
        rectShirt.height = (int)(SHIRT_SCALE_Y * rectFace.height);
        std::cout << "Shirt region is from " << rectShirt.x << ", " << rectShirt.y << " to " << rectShirt.x + rectShirt.width - 1
                << ", " << rectShirt.y + rectShirt.height - 1 << std::endl;

        // If the shirt region goes partly below the image, try just a little below the face
        bottom = rectShirt.y + rectShirt.height - 1;
        if (bottom > imageIn->height - 1) {
            SHIRT_DY = 0.95f; // Distance from top of face to top of shirt region, based on detected face height.
            SHIRT_SCALE_Y = 0.3f; // Height of shirt region compared to the detected face
            // Use a higher shirt region
            rectShirt.y = rectFace.y + (int)(SHIRT_DY * (float)rectFace.height)
                    + (int)(0.5f * (1.0f - SHIRT_SCALE_Y) * (float)rectFace.height);
            rectShirt.height = (int)(SHIRT_SCALE_Y * rectFace.height);
            initialConfidence = initialConfidence * 0.5f; // Since we are using a smaller region, we are less confident about the results now.
            std::cout << "Warning: Shirt region goes past the end of the image. Trying to reduce the shirt region position to "
                    << rectShirt.y << " with a height of " << rectShirt.height << std::endl;
        }

        // Try once again if it is partly below the image.
        bottom = rectShirt.y + rectShirt.height - 1;
        if (bottom > imageIn->height - 1) {
            bottom = imageIn->height - 1; // Limit the bottom
            rectShirt.height = bottom - (rectShirt.y - 1); // Adjust the height to use the new bottom
            initialConfidence = initialConfidence * 0.7f; // Since we are using a smaller region, we are less confident about the results now.
            std::cout
                    << "Warning: Shirt region still goes past the end of the image. Trying to reduce the shirt region height to "
                    << rectShirt.height << std::endl;
        }

        // Make sure the shirt region is in the image
        if (rectShirt.height <= 1) {
            std::cout << "Warning: Shirt region is not in the image at all, so skipping this face." << std::endl;
            msg.color[r] = "no_shirt";
            msg.safety[r] = 0.0;
        } else {

            // Show the shirt region
            drawRectangle(imageDisplay, rectShirt, CV_RGB(255, 255, 255));
            IplImage *imageShirt = cropRectangle(imageIn, rectShirt);
            IplImage *imageShirtHSV = cvCreateImage(cvGetSize(imageShirt), 8, 3);
            cvCvtColor(imageShirt, imageShirtHSV, CV_BGR2HSV); // (note that OpenCV stores RGB images in B,G,R order.
            if (!imageShirtHSV) {
                std::cerr << "ERROR: Couldn't convert Shirt image from BGR2HSV." << std::endl;
                exit(1);
            }

            int h = imageShirtHSV->height; // Pixel height
            int w = imageShirtHSV->width; // Pixel width
            int rowSize = imageShirtHSV->widthStep; // Size of row in bytes, including extra padding
            char *imOfs = imageShirtHSV->imageData; // Pointer to the start of the image HSV pixels.

            // Create an empty tally of pixel counts for each color type
            int tallyColors[NUM_COLOR_TYPES];
            for (int i = 0; i < NUM_COLOR_TYPES; i++)
                tallyColors[i] = 0;
            // Scan the shirt image to find the tally of pixel colors
            for (int y = 0; y < h; y++) {
                for (int x = 0; x < w; x++) {
                    // Get the HSV pixel components
                    uchar H = *(uchar*)(imOfs + y * rowSize + x * 3 + 0); // Hue
                    uchar S = *(uchar*)(imOfs + y * rowSize + x * 3 + 1); // Saturation
                    uchar V = *(uchar*)(imOfs + y * rowSize + x * 3 + 2); // Value (Brightness)

                    // Determine what type of color the HSV pixel is.
                    int ctype = getPixelColorType(H, S, V);
                    // Keep count of these colors.
                    tallyColors[ctype]++;
                }
            }

            // Print a report about color types, and find the max tally
            int tallyMaxIndex = 0;
            int tallyMaxCount = -1;
            int pixels = w * h;
            for (int i = 0; i < NUM_COLOR_TYPES; i++) {
                int v = tallyColors[i];
                std::cout << sCTypes[i] << " " << (v * 100 / pixels) << "%, ";
                if (v > tallyMaxCount) {
                    tallyMaxCount = tallyColors[i];
                    tallyMaxIndex = i;
                }
            }
            std::cout << std::endl;
            int percentage = initialConfidence * (tallyMaxCount * 100 / pixels);
            std::cout << "Color of shirt: " << sCTypes[tallyMaxIndex] << " (" << percentage << "% confidence)." << std::endl
                    << std::endl;

            msg.color[r] = sCTypes[tallyMaxIndex];
            msg.safety[r] = percentage;
            CvFont font;
            cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8, 1.0, 0, 1, CV_AA); // For OpenCV 2.0
            char text[256];
            cvPutText(imageDisplay, sCTypes[tallyMaxIndex], cvPoint(rectShirt.x, rectShirt.y + rectShirt.height + 12), &font,
                      CV_RGB(255, 0, 0));
            cvPutText(imageDisplay, text, cvPoint(rectShirt.x, rectShirt.y + rectShirt.height + 24), &font, CV_RGB(255, 0, 0));

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageDisplay).toImageMsg();
            imagePub_.publish(msg);
            cvReleaseImage(&imageShirtHSV);
            cvReleaseImage(&imageShirt);
        } //end if valid height
    } //end for loop
}

// Determine what type of color the HSV pixel is. Returns the colorType between 0 and NUM_COLOR_TYPES.
int ShirtDetector::getPixelColorType(int H, int S, int V) {
    int color;
    if (V < 75)
        color = cBLACK;
    else if (V > 190 && S < 27)
        color = cWHITE;
    else if (S < 53 && V < 185)
        color = cGREY;
    else { // Is a color
        if (H < 14)
            color = cRED;
        else if (H < 25)
            color = cORANGE;
        else if (H < 34)
            color = cYELLOW;
        else if (H < 73)
            color = cGREEN;
        else if (H < 102)
            color = cAQUA;
        else if (H < 127)
            color = cBLUE;
        else if (H < 149)
            color = cPURPLE;
        else if (H < 175)
            color = cPINK;
        else
            // full circle
            color = cRED; // back to Red
    }
    return color;
}
