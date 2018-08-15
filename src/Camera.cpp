#include "ros/ros.h"
#include "Minotauro/Trigger.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class LineDetection
{
    public:

        LineDetection()
		{
		}
        ~LineDetection()
		{
		}

        bool OpenCamera(int Camera_ID, int Cols, int Rows)
        {
            cols = Cols;
            rows = Rows;

            capture.set(CV_CAP_PROP_FRAME_WIDTH, cols);
            capture.set(CV_CAP_PROP_FRAME_WIDTH, rows);
            camera_ID = Camera_ID;

            capture.open(Camera_ID);

            if(capture.isOpened())
                return true;
            else
                return false;
        }

        int GetError()
        {
            capture.read(image);
            Mat partialImage = image(cv::Rect(0, rows*.495, cols, rows*.01));
            medianBlur(partialImage, partialImage, 11);
            Mat gray = Gray(partialImage);
            Mat binary = Binary(gray);
            binary.copyTo(partialImage);

            int nCount = 0;
            int nSum   = 0;

            ///Print a line in the half of rows
            for( int x = 0 ; x < image.cols ; x++ )
            {
                if(image.at<Vec3b>(image.rows*0.5, x)[0] == 0)
                {
                    image.at<Vec3b>(image.rows*0.5, x)[0] = 255;

                    nCount++;
                    nSum += x;
                }
                else
                   image.at<Vec3b>(240, x)[0] = 0;
            }
            ///Print a vertical line in the half of image
            for( int x = 0 ; x < image.rows ; x++ )
                image.at<Vec3b>(x,nSum/nCount)[0] = 255;

            //value = (nSum/nCount) - image.cols * .5;

            return (int) (nSum/nCount) - image.cols * .5;
        }

    private:

        int cols;
        int rows;
        int camera_ID;
        VideoCapture capture;
        //struct_Error error;
		Mat image;
    	//int value;

        Mat Gray(Mat src)
        {
            Mat		gray = src.clone();
            Vec3b	vColor;

            for (int y = 0; y < gray.rows; y++)
            {
                for (int x = 0; x < gray.cols; x++)
                {
                    // get pixel
                    vColor = gray.at<Vec3b>(Point(x, y));
                    vColor[0] = (uint8_t) vColor[0] * 0.2989 + vColor[1] * 0.5870 + vColor[2] * 0.1140;
                    vColor[1] = vColor[0];
                    vColor[2] = vColor[0];

                    // set pixel
                    gray.at<Vec3b>(Point(x, y)) = vColor;
                }
            }

            return gray;
        }
        Mat Binary(Mat src)
        {
            Mat	mOtsuMethod = src.clone();

            int	   iHistogram[256];
            double dHistogram[256];
            double dSum = 0;
            double dVariance[256];
            double dVarianceMayor = 0;
            int	   ikMayor = 0;
            double dPk = 0;
            double dmk = 0;
            double dmG = 0;

            Vec3b	vColor;

            //Clean histogram
            for (int x = 0; x < 256; x++)
            {
                iHistogram[x] = 0;
                dHistogram[x] = 0;
                dVariance[x]   = 0;
            }

            // Counts the intensity in any pixel
            for (int y = 0; y < mOtsuMethod.rows; y++)
            {
                for (int x = 0; x < mOtsuMethod.cols; x++)
                {
                    // get pixel
                    vColor = mOtsuMethod.at<Vec3b>(Point(x, y));
                    iHistogram[vColor[0]]++;
                }
            }

            // Normalize histogram
            for (int x = 0; x < 256; x++)
            {
                dHistogram[x] = (double)iHistogram[x] / (mOtsuMethod.cols*mOtsuMethod.rows);
                dSum += dHistogram[x];
            }

            // Get mG
            for (int x = 0; x < 256; x++)
            {
                dmG += x * dHistogram[x];
            }

            for (int k = 0; k < 256; k++)
            {
                dPk = 0;
                dmk = 0;

                // Get P(k), m(k)
                for (int x = 0; x <= k; x++)
                {
                    dPk += dHistogram[x];
                    dmk += x * dHistogram[x];
                }
                // Get variance
                dVariance[k] = (((dmG * dPk) - dmk) * ((dmG * dPk) - dmk)) / (dPk * (1 - dPk));

                // Get variance and the bigger K
                if (dVariance[k] > dVarianceMayor)
                {
                    dVarianceMayor = dVariance[k];
                    ikMayor = k;
                }
            }

            // Counts the intensity in any pixel
            for (int y = 0; y < mOtsuMethod.rows; y++)
            {
                for (int x = 0; x < mOtsuMethod.cols; x++)
                {
                    // get pixel
                    vColor = mOtsuMethod.at<Vec3b>(Point(x, y));

                    // If the average of the intensities is greater or equal than K, the maximum intensity is assigned
                    if (vColor[0] >= ikMayor)
                    {
                        vColor[0] = 255;
                        vColor[1] = 255;
                        vColor[2] = 255;
                    }
                    // If not the minimum intensity is assigned
                    else
                    {
                        vColor[0] = 0;
                        vColor[1] = 0;
                        vColor[2] = 0;
                    }
                    mOtsuMethod.at<Vec3b>(Point(x, y)) = vColor;
                }
            }

            return mOtsuMethod;
        }
};

LineDetection lineDetection;

bool CameraTrigger_Callback(Minotauro::Trigger::Request  &req, Minotauro::Trigger::Response &res)
{
  	res.CameraError = lineDetection.GetError();

  	ROS_INFO("Camera - Trigger received");
  	ROS_INFO("Camera - Sending back response: %d", (int) res.CameraError);

  	return true;
}

int main(int argc, char **argv)
{
	lineDetection.OpenCamera(0, 640, 480);

  	ros::init(argc, argv, "Camera_Node");
  	ros::NodeHandle n;

  	ros::ServiceServer CameraTrigger_Server = n.advertiseService("CameraTrigger_Service", CameraTrigger_Callback);

  	ROS_INFO("Camera - Ready to receive camera trigger request");
  	ros::spin();

  	return 0;
}
