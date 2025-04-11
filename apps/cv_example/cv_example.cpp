#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		//printf("usage: DisplayImage.out <Image_Path>\n");
		std::cout << "USAGE: " << argv[0] << " <Image_Path>" << std::endl;
		return 1;
	}

	Mat image;
	image = imread(argv[1], IMREAD_COLOR);

	if (!image.data)
	{
		printf("No image data \n");
		return 2;
	}
	namedWindow("Display Image", WINDOW_AUTOSIZE);
	imshow("Display Image", image);

	waitKey(0);

	return 0;
}