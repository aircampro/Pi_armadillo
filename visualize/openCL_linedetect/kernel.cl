// kernel.cl this is the kernel program which does the line detection 

// This is the preferred kernel function.In this program, this kernel function starts 640×480=307200 threads 
// and calculates line segment detection for each pixel.The method of line segment detection is very simple, 
// with respect to the 3×3 region around the detection target pixel, it is possible to adjust the coefficient 
// so that the higher the value as the center is bright and the periphery is dark.You just need to add them together.
// Pixels with uniform brightness cancel each other and become zero, but pixels above points and lines get a value greater 
// than zero.For details, please investigate the antagonistic receptor field around the center of the retina

__kernel void detectLine(__global unsigned char* result, __global unsigned char* origin, int2 size)
{
	// get_global_id is a function to get the number of threads executed in parallel
	int x = get_global_id(0);
	int y = get_global_id(1);

	float val = 0;
	val += 6.828 * origin[(size.x * y + x) * 3];
	val -= origin[(size.x * y + (x - 1)) * 3];
	val -= origin[(size.x * y + (x + 1)) * 3];
	val -= origin[(size.x * (y - 1) + x) * 3];
	val -= origin[(size.x * (y + 1) + x) * 3];
	val -= 0.707 * origin[(size.x * (y - 1) + (x - 1)) * 3];
	val -= 0.707 * origin[(size.x * (y - 1) + (x + 1)) * 3];
	val -= 0.707 * origin[(size.x * (y + 1) + (x - 1)) * 3];
	val -= 0.707 * origin[(size.x * (y + 1) + (x + 1)) * 3];
	result[size.x * y + x] = (unsigned char)val;
}