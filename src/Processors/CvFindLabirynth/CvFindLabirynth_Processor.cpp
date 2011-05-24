#include "CvFindLabirynth_Processor.hpp"

#include "LReading.hpp"

#include <memory>
#include <string>

#include "Common/Logger.hpp"


namespace Processors {

namespace CvFindLabirynth {

using namespace std;
using namespace boost;
using namespace cv;
using namespace Types::Objects3D;
using Types::Mrrocpp_Proxy::LReading;

CvFindLabirynth_Processor::CvFindLabirynth_Processor(const std::string & name) :
	Component(name)
{
	has_image = false;
	q=0;
	path_exists = false;
	path_set = false;
	corner_LU = cv::Point2i(0,0);
	corner_RD = cv::Point2i(0,0);


	findLabirynthFlags = 0;
	temp=90;
	found=true;

}

CvFindLabirynth_Processor::~CvFindLabirynth_Processor()
{
}

bool CvFindLabirynth_Processor::onFinish()
{
	return true;
}

bool CvFindLabirynth_Processor::onStop()
{
	return true;
}

bool CvFindLabirynth_Processor::onInit()
{
	h_onNewImage.setup(this, &CvFindLabirynth_Processor::onNewImage);
	registerHandler("onNewImage", &h_onNewImage);
	h_onRpcCall.setup(this, &CvFindLabirynth_Processor::onRpcCall);
	registerHandler("onRpcCall", &h_onRpcCall);

	registerStream("in_img", &in_img);
	registerStream("rpcParam", &rpcParam);

	registerStream("rpcResult", &out_info);

	rpcResult = registerEvent("rpcResult");

	//changed image
	newImage = registerEvent("newImage");
	registerStream("out_img", &out_img);

	LOG(LTRACE) << "component initialized\n";
	return true;
}

bool CvFindLabirynth_Processor::onStart()
{
	return true;
}

bool CvFindLabirynth_Processor::onStep()
{
	return true;
}

void CvFindLabirynth_Processor::onRpcCall()
{
	LOG(LTRACE) << "void CvFindLabirynth_Processor::onRpcCall() begin\n";

	xdr_iarchive <> param = rpcParam.read();
	double paramT;
	param >> paramT;
	LOG(LNOTICE) << "CvFindLabirynth_Processor::onRpcCall(): paramT=" << paramT;

	LReading lr;

	//!
	lr.path_exists = path_exists;

	//init
	for(int i=0;i<50;i++)
		for(int j=0;j<2;j++)
			lr.path[i][j] = -1;

	std::vector<Point_labirynth>::reverse_iterator rit;//reverse_
	int ix;

	for ( rit=path.rbegin(), ix=0 ; rit < path.rend(); ++rit, ++ix )
	{
		lr.path[ix][0] = (*rit).x;
		lr.path[ix][1] = (*rit).y;
		std::cout<<"POINT: "<<(*rit).x<<","<<(*rit).y<<std::endl;
	}

	for(int i=0;i<50;i++)
	{
		for(int j=0;j<2;j++)
			std::cout<<"LR.path: "<<lr.path[i][0]<<","<<lr.path[i][1]<<std::endl;
	}

	out_info.write(lr);

	//std::cout<<"lr.info= "<<lr.info<<std::endl;
/*
	cv::Point2f point((image.size().width / 2),(image.size().height / 2));
	corners.push_back(point);
	corners.push_back(point);


	Types::ImagePosition imagePosition;
	double maxPixels = std::max(image.size().width, image.size().height);
	imagePosition.elements[0] = (corners[0].x - image.size().width / 2) / maxPixels;
	imagePosition.elements[1] = (corners[0].y - image.size().height / 2) / maxPixels;
	imagePosition.elements[2] = 0;
	imagePosition.elements[3] = - atan2(corners[1].y - corners[0].y, corners[1].x - corners[0].x);
	out_imagePosition.write(imagePosition);

	lr.info = "info3";
	out_info.write(lr);

	labirynthFound->raise();

	corners.pop_back();
	corners.pop_back();
*/

	rpcResult->raise();
}


void CvFindLabirynth_Processor::onNewImage()
{
	LOG(LTRACE) << "void CvFindLabirynth_Processor::onNewImage() begin\n";
	try
	{
		if(in_img.empty())
		{
			LOG(LWARNING) << "in_img empty!";
			return;
		}

		image = in_img.read();

		/*do it once only*/
		if(!has_image)
		{

			int x_rect=0;
			int y_rect=0;
			//int eps = 4;

			// Read image from input data stream.
			cv::Mat img = image;
			/* create new image for the grayscale version */
			cv::Mat dst(img.rows,img.cols, img.depth());
			/* CV_RGB2GRAY: convert RGB image to grayscale */
			cv::cvtColor( img, dst, CV_RGB2GRAY );

			//cv::vector<cv::Vec3f> circles;
			//cv::HoughCircles(dst,circles, CV_HOUGH_GRADIENT, 1.15, 100, 200);

			//while(!circles.empty())
			//{
			//	std::cout<<"cir: "<<circles.back()[0]<<" "<<circles.back()[1]<<" "<<circles.back()[2]<<endl;
				//ball_center.x = circles.back()[0];
				//ball_center.y = circles.back()[1];
			//			circles.pop_back();
			//}

			/* color invert	*/
			cv::bitwise_not(dst,dst);

			/* contour joy */
			cv::Mat contour(dst.rows,dst.cols, dst.depth());
			cv::Mat helper(dst.rows,dst.cols, dst.depth());
			cv::threshold(dst,contour,70,255,CV_THRESH_BINARY);



			int minx=500, miny=500,maxx=0,maxy=0;

			cv::bitwise_not(contour,contour);
			cv::vector<cv::Vec4i> lines;
			cv::HoughLinesP(contour, lines, 1, CV_PI/2, 220, 150, 0);//src,contener,?,kat,treshold,minlength//240,210
			for( size_t i = 0; i < lines.size(); i++ )
			{
				//x
				if(lines[i][0]<minx)
					minx=lines[i][0];
				if(lines[i][2]<minx)
					minx=lines[i][2];
				if(lines[i][0]>maxx)
					maxx=lines[i][0];
				if(lines[i][2]>maxx)
					maxx=lines[i][2];
				//y
				if(lines[i][1]<miny)
					miny=lines[i][1];
				if(lines[i][3]<miny)
					miny=lines[i][3];
				if(lines[i][1]>maxy)
					maxy=lines[i][1];
				if(lines[i][3]>maxy)
					maxy=lines[i][3];

				cv::line( img, cv::Point(lines[i][0], lines[i][1]),
					cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 1, 8 );
			}
			cv::bitwise_not(contour,contour);

			cout<<"LU: "<<minx<<", "<< miny<<" RD: "<<maxx<<" "<<maxy<<endl;

			/* find labirynth's walls */
			//find_labirynth(contour, helper,0,1);
			//find_labirynth(contour, helper,0,-1);
			//find_labirynth(contour, helper,1,1);
			//find_labirynth(contour, helper,1,-1);

			corner_LU.x = minx;
			corner_LU.y = miny;
			corner_RD.x = maxx;
			corner_RD.y = maxy;
			std::cout<<corner_LU.x<<" "<<corner_LU.y<<" "<<corner_RD.x<<" "<<corner_RD.y<<endl;

			//DEFINEs
			EPS = 5;
			int temp1, temp2;
			temp1 = (corner_RD.x - corner_LU.x)/LABIRYNT_SIZE_X;
			temp2 = (corner_RD.y - corner_LU.y)/LABIRYNT_SIZE_Y;
			FIELD = (temp1 + temp2)/2;//27
			WALL = FIELD/2;


			/* find walls inside labirynth */
			cv::Point2i corner((corner_LU.x + WALL),(corner_LU.y + WALL));

			for(y_rect=0; y_rect < LABIRYNT_SIZE_Y; y_rect++)
			{
				for(x_rect=0; x_rect < LABIRYNT_SIZE_X; x_rect++)
				{
					//dzielenie int'ow?
					cv::Point2i center(corner.x + (x_rect)*((corner_RD.x - corner_LU.x)/LABIRYNT_SIZE_X), corner.y + (y_rect)*((corner_RD.y - corner_LU.y)/LABIRYNT_SIZE_Y));

					printf("POLE (%d, %d)\n",x_rect+1,y_rect+1);

					cv::Point2i search = center;

					/**/

					cv::Scalar pix;
					pix.val[0] = contour.at<uchar>(search.y, search.x);

					if(pix.val[0] != 255.0)//TRAFILISMY NA KULKE
					{
						cv::Point2i pt1, pt2;
						int e = 5;
						pt1.x = search.x - (WALL-e);
						pt1.y = search.y - (WALL-e);
						pt2.x = search.x + (WALL-e);
						pt2.y = search.y + (WALL-e);

						rectangle(contour, pt1, pt2, 255.0/*const Scalar& color*/,CV_FILLED /* int thickness=1, int lineType=8, int shift=0*/);
						ball_center.x = search.x;
						ball_center.y = search.y;
					}



					/**/

					labirynt_info_array[y_rect][x_rect].n_wall = find_wall(contour, helper, 1,-1, center);
					labirynt_info_array[y_rect][x_rect].w_wall = find_wall(contour, helper, 0,-1, center);
					labirynt_info_array[y_rect][x_rect].e_wall = find_wall(contour, helper, 0, 1, center);
					labirynt_info_array[y_rect][x_rect].s_wall = find_wall(contour, helper, 1, 1, center);
					//to powinno byc w konstruktorze
					labirynt_info_array[y_rect][x_rect].value = -1;

					//cout<<labirynt_info_array[y_rect][x_rect].n_wall<<endl;
				}
			}

			//find start field (knowing ball_center point)
			int FIELD_X = ((corner_RD.x - corner_LU.x)/LABIRYNT_SIZE_X);
			int FIELD_Y = ((corner_RD.y - corner_LU.y)/LABIRYNT_SIZE_Y);

			int xfield,yfield;

			cv::Point2i temp = corner;

			for(int ny=0;ny<LABIRYNT_SIZE_Y;ny++)
			{
				if(ball_center.y+FIELD_Y/2 > temp.y && ball_center.y-FIELD_Y/2 < temp.y)
				{
					std::cout<<"temp.y"<<temp.y<<endl;
					yfield = ny;
					break;
				}
				temp.y+=FIELD_Y;
			}
			temp = corner;
			for(int nx=0;nx<LABIRYNT_SIZE_X;nx++)
			{
				if(ball_center.x+FIELD_X/2 > temp.x && ball_center.x-FIELD_X/2 < temp.x)
				{
					std::cout<<"temp.x"<<temp.x<<endl;
					xfield = nx;
					break;
				}
				temp.x+=FIELD_X;
			}
			std::cout<<"FIELD: "<<xfield<<" "<<yfield<<endl;


			/* find path */
			if (find_path(xfield,yfield) == false)
				printf("Labirynt nie ma drogi wyjscia!!\n");
			else
			{
				printf("Znaleziono wyjscie - Oto sciezka:\n");
				for(unsigned int i=0;i<path.size();i++)
				{
					printf("%d,%d\n",path[i].x,path[i].y);
				}
			}


			has_image = true;
			out_img.write(contour);//contour / image
			newImage->raise();

		}//has_image
		else
		{
			//image catched
		}


	} catch (const Exception& e) {
		LOG(LERROR) << e.what() << "\n";
	}
	LOG(LTRACE) << "void CvFindLabirynth_Processor::onNewImage() end\n";
}

/**
 * find_wall from center point
 * @param IMG
 * @param flag 0 change x 1 change y
 * @param value 1 (down) or -1 (up)
 * @param center - start point
 */
bool CvFindLabirynth_Processor::find_wall(cv::Mat img, cv::Mat dst, int flag, int value, cv::Point2i center)
{
	cv::Point2i search = center;
	cv::Scalar pix;

	for(int i=1;i<(WALL+3*EPS);i++)
	{

		pix.val[0] = img.at<uchar>(search.y, search.x);

		if(pix.val[0] != 255.0)
		{
			/*wydruki*/
			//std::cout<<search.x<<" "<<search.y<<endl;
			if(flag==0)
			{
				if(value==1)
					printf("SCIANA: NA PRAWO\n");
				else
					printf("SCIANA: NA LEWO\n");
			}
			if(flag==1)
			{
				if(value==1)
					printf("SCIANA: NA DOLE\n");
				else
					printf("SCIANA: NA GORZE\n");
			}

			/*koniec wydrukow*/

			return true;
		}

		/* rysowanie sciezki poszukiwan */
		//pix.val[0] = 0;
		//cvSet2D(dst,search.y,search.x,pix);
		//dst.at<int>(search) = pix[0];//powinno byc tylko pix!

		if(flag == 0)
			search.x = search.x + value;
		if(flag == 1)
			search.y = search.y + value;
	}
return false;
}
/**
 * find path in labirynth using labirynt_info_array
 */
bool CvFindLabirynth_Processor::find_path(int x, int y)
{
	Point_labirynth start_point;
	start_point.x = x;
	start_point.y = y;
	labirynt_info_array[y][x].value = 0;

	set_neighbours(y,x);

	/*wydruk*/
	printf("JESTEM W: %d,%d\n",x,y);
	for(int i=0;i<LABIRYNT_SIZE_Y;i++)
	{
		for(int ii=0;ii<LABIRYNT_SIZE_X;ii++)
			printf("%d ",labirynt_info_array[i][ii].value);

		printf("\n");
	}
	printf("\n");
	/* * */

	Point_labirynth end_point;
	end_point.x = LABIRYNT_SIZE_X-1;
	end_point.y = LABIRYNT_SIZE_Y-1;
	if(path_exists)
		set_path(end_point,start_point);

return path_exists;
}
/**
 * set numbers in labirynt_info_array
 */
void CvFindLabirynth_Processor::set_neighbours(int y, int x)
{
	/* exit exsists*/
	if((x == LABIRYNT_SIZE_X-1) && (y == LABIRYNT_SIZE_Y-1))
	{
		path_exists = true;
		return ;
	}

	/* zapisz liczby na sasiadach jezeli jeszcze nie odwiedzone (-1) lub odwiedzone po dluzszej sciezce*/

	if(labirynt_info_array[y][x].e_wall == false)
	{
		if((labirynt_info_array[y][x+1].value < 0) || (labirynt_info_array[y][x+1].value > labirynt_info_array[y][x].value))
			labirynt_info_array[y][x+1].value = labirynt_info_array[y][x].value + 1;
	}
	if(labirynt_info_array[y][x].s_wall == false)
	{
		if((labirynt_info_array[y+1][x].value < 0) || (labirynt_info_array[y+1][x].value > labirynt_info_array[y][x].value))
			labirynt_info_array[y+1][x].value = labirynt_info_array[y][x].value + 1;
	}
	if(labirynt_info_array[y][x].w_wall == false)
	{
		if((labirynt_info_array[y][x-1].value < 0) || (labirynt_info_array[y][x-1].value > labirynt_info_array[y][x].value))
			labirynt_info_array[y][x-1].value = labirynt_info_array[y][x].value + 1;
	}
	if(labirynt_info_array[y][x].n_wall == false)
	{
		if((labirynt_info_array[y-1][x].value < 0) || (labirynt_info_array[y-1][x].value > labirynt_info_array[y][x].value))
			labirynt_info_array[y-1][x].value = labirynt_info_array[y][x].value + 1;
	}

	/* poruszaj sie dalej */

	if((labirynt_info_array[y][x].e_wall == false) && (labirynt_info_array[y][x+1].value == labirynt_info_array[y][x].value+1))
		set_neighbours(y,x+1);

	if((labirynt_info_array[y][x].s_wall == false) && (labirynt_info_array[y+1][x].value == labirynt_info_array[y][x].value+1))
		set_neighbours(y+1,x);

	if((labirynt_info_array[y][x].w_wall == false) && (labirynt_info_array[y][x-1].value == labirynt_info_array[y][x].value+1))
		set_neighbours(y,x-1);

	if((labirynt_info_array[y][x].n_wall == false) && (labirynt_info_array[y-1][x].value == labirynt_info_array[y][x].value+1))
		set_neighbours(y-1,x);

return ;
}
/**
 * set path in "path" vector
 */
void CvFindLabirynth_Processor::set_path(Point_labirynth p,Point_labirynth s)
{
	path.push_back(p);

	if((p.x==s.x) && (p.y==s.y))
	{
		path_set = true;
		return ;
	}

	if((labirynt_info_array[p.y][p.x].n_wall == false) && (labirynt_info_array[p.y-1][p.x].value == labirynt_info_array[p.y][p.x].value-1))
	{
		p.y = p.y-1;
		set_path(p,s);
		if(path_set)
			return ;
	}
	if((labirynt_info_array[p.y][p.x].w_wall == false) && (labirynt_info_array[p.y][p.x-1].value == labirynt_info_array[p.y][p.x].value-1))
	{
		p.x = p.x-1;
		set_path(p,s);
		if(path_set)
			return ;

	}
	if((labirynt_info_array[p.y][p.x].s_wall == false) && (labirynt_info_array[p.y+1][p.x].value == labirynt_info_array[p.y][p.x].value-1))
	{
		p.y = p.y+1;
		set_path(p,s);
		if(path_set)
			return ;
	}
	if((labirynt_info_array[p.y][p.x].e_wall == false) && (labirynt_info_array[p.y][p.x+1].value == labirynt_info_array[p.y][p.x].value-1))
	{
		p.x = p.x+1;
		set_path(p,s);
		if(path_set)
			return ;
	}
return ;
}



} // namespace CvFindLabirynth {
} // namespace Processors {
