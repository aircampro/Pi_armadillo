#include "opencv2/highgui.hpp"
#include "opencv2/Videoio.hpp"
#include <opencv2/core.hpp>

#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(){

	VideoWriter writer("/home/anthony/vid_test/OutVideo.wmv", VideoWriter::fourcc('W', 'M', 'V', '1'), 15.0, Size(1024, 1024));

	if (!writer.isOpened()){ return -1; }


	char image_name[100];
	Mat image;

	// from 0000000.jpg to 0001000.jpg
	for (int i = 0; i < 1001; i++){
		// make the filenames
		sprintf(image_name, "C:\\log\\%07d.jpg", i);
		image = imread(image_name);

		// no image or invalid then skip
		if (image.empty()) {
			cout << "no image : " << image_name << endl;
			continue;
		}
        // write to the video
		writer << image;
	}
	return 0;
}