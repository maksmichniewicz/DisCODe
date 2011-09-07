#include "CvFindLabirynth_Processor.hpp"

#include "LReading.hpp"

#include <memory>
#include <string>

#include "Common/Logger.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


namespace Processors {

namespace CvFindLabirynth {

using namespace std;
using namespace boost;
using namespace cv;
using namespace Types::Objects3D;
using Types::Mrrocpp_Proxy::LReading;

CvFindLabirynth_Processor::CvFindLabirynth_Processor(const std::string & name) :
	Component(name),
	m_param1("param1", 192, "range"),
	m_param2("param2", 20, "range")
{
	has_image = false;
	q=0;
	path_exists = false;
	path_set = false;
	corner_LU = cv::Point2i(0,0);
	corner_RD = cv::Point2i(0,0);
	path.clear();

	m_param1.addConstraint("0");
	m_param1.addConstraint("255");

	m_param2.addConstraint("0");
	m_param2.addConstraint("255");

	registerProperty(m_param1);
	registerProperty(m_param2);

//	findLabirynthFlags = 0;
//	temp=90;
//	found=true;
//
//	TEMP = 0;
//	HELP = false;

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
	lr.waiting = false;


	if(paramT>0.0)
		has_image = false;

	//jezeli nie jest to pierwsze czytanie to czytaj raz jeszcze!
	if(!has_image)
	{

		LOG(LNOTICE) << "Nowe przetworzenie zdjecia...";
		lr.waiting = true;
		out_info.write(lr);
		rpcResult->raise();
		LOG(LNOTICE) << "Koniec przetworzenia, przesylam dane z waiting";
		return;
	}

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
		std::cout<<"LR.path: "<<lr.path[i][0]<<","<<lr.path[i][1]<<std::endl;
	}

	out_info.write(lr);
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

		/*do it once only*/
		if(!has_image)
		{
			//czysc dane zeby bylo ok przed nastepnym uzyciem
			//has_image = false;
			q=0;
			path_exists = false;
			path_set = false;
			corner_LU = cv::Point2i(0,0);
			corner_RD = cv::Point2i(0,0);

			path.clear();
			//********************************



			image = in_img.read();
			//has_image = true;

			int x_rect=0;
			int y_rect=0;
			//int eps = 4;

/***********************/
/** MAKE CONTOUR IMAGE */
/***********************/

			// Read image from input data stream.
			cv::Mat img = image;
			/* create new image for the grayscale version */
			cv::Mat dst(img.rows,img.cols, img.depth());
			/* CV_RGB2GRAY: convert RGB image to grayscale */
			cv::cvtColor( img, dst, CV_RGB2GRAY );
			/* color invert	*/
			cv::bitwise_not(dst,dst);

			/* contour joy */
			cv::Mat contour(dst.rows,dst.cols, dst.depth());
			cv::Mat helper(dst.rows,dst.cols, dst.depth());
			cv::threshold(dst,contour,70,255,CV_THRESH_BINARY);

			cv::Mat dst_rotate;
			cv::Mat img_rotate;

/***********************/
/** FIND OUTSIDE WALLS */
/***********************/

			int minx=1000, miny=1000,maxx=0,maxy=0;
			float angle = 0;

			std::cout<<"Przetwarzanie obrazu..."<<std::endl;

			while(angle>-360)
			{
				minx=1000;
				miny=1000;
				maxx=0;
				maxy=0;
				/***********************/
				/* rotacja */
				/***********************/
				CvPoint2D32f centre;
				CvMat *translate_matrix = cvCreateMat(2, 3, CV_32FC1);

				cvSetZero(translate_matrix);
				centre.x =  contour.cols/2;
				centre.y = contour.rows/2;
				cv2DRotationMatrix(centre, angle, 1.0, translate_matrix);

				int ss = sqrt(contour.cols*contour.cols + contour.rows*contour.rows);

				cvmSet(translate_matrix, 0, 2, cvmGet(translate_matrix,0,2)+(ss-contour.cols)/2);
				cvmSet(translate_matrix, 1, 2, cvmGet(translate_matrix,1,2)+(ss-contour.rows)/2);

				//cv::Mat dst_rotate(ss, ss, contour.depth());
				//cv::Mat img_rotate(ss, ss, img.depth());

				cv::warpAffine(contour, dst_rotate, translate_matrix, cv::Size(ss,ss),INTER_LINEAR, BORDER_CONSTANT, 255);//, CV_INTER_LINEAR , CV_WARP_FILL_OUTLIERS, 0);
				cv::warpAffine(img, img_rotate, translate_matrix, cv::Size(ss,ss),INTER_LINEAR, BORDER_CONSTANT, 255);//, CV_INTER_LINEAR , CV_WARP_FILL_OUTLIERS, 0);

				cvReleaseMat(&translate_matrix);

				/***********************/
				/* szukanie scian */
				/***********************/

				cv::bitwise_not(dst_rotate,dst_rotate);
				cv::vector<cv::Vec4i> lines;
				cv::HoughLinesP(dst_rotate, lines, 1, CV_PI/2, 220, 240, 0);//src,contener,?,kat,treshold,minlength//240,210
				for( size_t i = 0; i < lines.size(); i++ )
				{
					//std::cout<<"HoughLinesP : lines.size()!=0 "<<endl;
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

					cv::line( img_rotate, cv::Point(lines[i][0], lines[i][1]),
						cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 1, 8 );
				}
				cv::bitwise_not(dst_rotate,dst_rotate);
				boost::this_thread::sleep(boost::posix_time::milliseconds(50));

				/***********************/
				/* sprawdzenie poprawnosci w danym ulozeniu */
				/***********************/

				if(lines.size() != 0 && maxx-minx>contour.cols*(2/3) && maxy-miny>contour.rows*(2/3))//!!! zamienic te stale!
				{
					cv::line( img_rotate, cv::Point(minx,miny),
							cv::Point(maxx, maxy), cv::Scalar(0,255,0), 1, 8 );

					std::cout<<"OK! Znaleziono sciany, czy jest wyjscie gdzie trzeba?"<<std::endl;

					cv::Scalar pix;
					int is_gate = -1;

					int field = (maxx-minx)/LABIRYNT_SIZE_X;

					for(int i=0;i<field;i++)//!!! 10 i 10/2 -> nie znamy jeszcze wielkosci FIELD..w kazdym razie sprawdzamy czy przy dolnym prawym narozniku po jego lewej nie ma scianki
					{

						pix.val[0] = dst_rotate.at<uchar>(maxy-field/2+i,maxx-field/2);//(Y:rows,X:cols)!!!!!!!

						//std::cout<<" PIX przed: "<<pix.val[0]<<" "<<maxx-field/2<<" "<<maxy-field/2+i<<std::endl;
						cv::line( img_rotate, cv::Point(maxx-field/2,maxy-field/2+i),
							cv::Point(maxx-field/2,maxy-field/2+i), 0, 1, 8 );
						//std::cout<<" PIX po: "<<pix.val[0]<<" "<<maxx-field/2<<" "<<maxy-field/2+i<<std::endl;

						if(pix.val[0] < 255.0)
						{

							std::cout<<"Nie, jest tam sciana: "<<maxx-field/2<<" "<<maxy-field/2+i<<std::endl;
							is_gate = 0;
							angle-=88;//jezeli juz znalazl sciany to obroc o 90 a nie dalej co dwa stopnie..
							break;
						}
						is_gate = 1;
					}

					std::cout<<"is_gate: "<<is_gate<<std::endl;

					if(is_gate>0)
					{
						std::cout<<"OK! Labirynt odnaleziony, wyjscie tez"<<std::endl;
						break;
					}
				}

				out_img.write(img_rotate);//contour / image
				newImage->raise();


			angle-=2;
			}//while(360stopni)
			if(angle<=-360)
			{std::cout<<"BOKI LABIRYNTU NIE ZNALEZIONE!!! SPRAWDZ CZY NA ZDJECIU JEST OBRAZ CALEGO LABIRYNTU I CZY NIE JEST ON ZBYT MALY"<<std::endl;}

			corner_LU.x = minx;
			corner_LU.y = miny;
			corner_RD.x = maxx;
			corner_RD.y = maxy;
			std::cout<<"LU: "<<corner_LU.x<<" "<<corner_LU.y<<" RD: "<<corner_RD.x<<" "<<corner_RD.y<<endl;

			//DEFINEs
			EPS = 5;
			int temp1, temp2;
			temp1 = (corner_RD.x - corner_LU.x)/LABIRYNT_SIZE_X;
			temp2 = (corner_RD.y - corner_LU.y)/LABIRYNT_SIZE_Y;
			FIELD = (temp1 + temp2)/2;//27
			WALL = FIELD/2;
			std::cout<<"FIELD: "<<FIELD<<std::endl;

			//czyszczenie szumow na zewnatrz labiryntu
			for(int i=0;i<corner_LU.y-5;i++)
			{
				cv::line( dst_rotate, cv::Point(0,i), cv::Point(dst_rotate.cols,i), 255, 1, 8 );
			}
			for(int i=corner_RD.y+5;i<dst_rotate.rows;i++)
			{
				cv::line( dst_rotate, cv::Point(0,i), cv::Point(dst_rotate.cols, i), 255, 1, 8 );
			}
			for(int i=0;i<corner_LU.x-5;i++)
			{
				cv::line( dst_rotate, cv::Point(i,0), cv::Point(i,dst_rotate.rows), 255, 1, 8 );
			}
			for(int i=corner_RD.x+5;i<dst_rotate.cols;i++)
			{
				cv::line( dst_rotate, cv::Point(i,0), cv::Point(i,dst_rotate.rows), 255, 1, 8 );
			}


/***********************/
/** FIND BALL AND HIDE */
/***********************/

			cv::Mat blr(dst_rotate.rows,dst_rotate.cols, dst_rotate.depth());;
			GaussianBlur( dst_rotate, blr, Size(9, 9), 2, 2 );

			cv::vector<cv::Vec3f> circles;
			bool is_ball=false;


			//cv::HoughCircles(blr, circles, CV_HOUGH_GRADIENT,
			//2/*1*/, FIELD/*10*/, 10/*75*/, 10/*50-jj*/, FIELD/2-6, FIELD/2-2/*-2-xx*/);//4, 15);// FIELD/4, FIELD/3);

			int x=25;
			while(x>9)
			{
			cv::HoughCircles(blr, circles, CV_HOUGH_GRADIENT, 2, FIELD, m_param1, x/*m_param2*/, FIELD/2-8/*FIELD/2-6*/, FIELD/2-1);

			/*if(circles.size()!=0)*/for( size_t i = 0; i < circles.size(); i++ )
			{
				 std::cout<<"CIRCLE ii: "<<circles[i][0]<<" "<<circles[i][1]<<" "<<circles[i][2]<<endl;

				 circles[i][0] += 1;//wsp x
				 circles[i][1] += 1;//wsp y
				 circles[i][2] += 5;//promien - zamaluj wiecej



				cv::Scalar pixy;
				pixy.val[0] = blr.at<uchar>(circles[i][1], circles[i][0]);
				if(pixy.val[0] < 250.0)
				{
					ball_center.x = circles[i][0];
					ball_center.y = circles[i][1];
					std::cout<<"CIRCLE: GOOD "<<circles[i][0]<<" "<<circles[i][1]<<" "<<circles[i][2]<<endl;
					is_ball=true;
				}


				 cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				 int radius = cvRound(circles[i][2]);
				 //inne zle okregi
				 //cv::circle( dst_rotate, center, radius, Scalar(200,200,200), -1, 8, 0 );


				 if(pixy.val[0] < 250.0)
				 {
					 cv::circle( dst_rotate, center, radius, Scalar(255,255,255), -1, 8, 0 );
					 std::cout<<"PARAM2 = "<<x<<endl;
					 x=0;
				 }

			}

			x--;
			}


			//out_img.write(dst_rotate);//contour / image
			//newImage->raise();
			//boost::this_thread::sleep(boost::posix_time::milliseconds(1000));


			if (!is_ball)
			{
				std::cout<<"PROBLEM WITH BALL DETECTION! RESTART"<<std::endl;
				return;
			}
			std::cout<<"END"<<std::endl;



//			std::cout<<"FIELD: "<<FIELD<<std::endl;
//			std::cout<<"CIRCLES:"<<std::endl;
//
//			if(circles.size() != 0)
//			{
//				circles[0][0] += 2;//zamaluj wiecej
//				circles[0][1] += 2;
//				circles[0][2] += 7;//radius - poprawka
//
//				ball_center.x = circles[0][0];
//				ball_center.y = circles[0][1];
//
//				for( size_t i = 0; i < circles.size(); i++ )
//					{
//						 cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//						 int radius = cvRound(circles[i][2]);
//
//						 std::cout<<"CIRCLE: "<<circles[i][0]<<" "<<circles[i][1]<<" "<<circles[i][2]<<endl;
//
//						 cv::circle( dst_rotate, center, radius, Scalar(255,255,255), -1, 8, 0 );
//					}
//
//
//				std::cout<<"CIRCLES END:"<<std::endl;
//
//
/***********************/
/** FIND INSIDE WALLS */
/***********************/
				cv::Point2i corner((corner_LU.x + WALL),(corner_LU.y + WALL));

				for(y_rect=0; y_rect < LABIRYNT_SIZE_Y; y_rect++)
				{
					for(x_rect=0; x_rect < LABIRYNT_SIZE_X; x_rect++)
					{
						//dzielenie int'ow?
						cv::Point2i center(corner.x + (x_rect)*((corner_RD.x - corner_LU.x)/LABIRYNT_SIZE_X), corner.y + (y_rect)*((corner_RD.y - corner_LU.y)/LABIRYNT_SIZE_Y));

						//printf("POLE (%d, %d)\n",x_rect+1,y_rect+1);

						cv::Point2i search = center;


						labirynt_info_array[y_rect][x_rect].n_wall = find_wall(dst_rotate, helper, 1,-1, center);
						labirynt_info_array[y_rect][x_rect].w_wall = find_wall(dst_rotate, helper, 0,-1, center);
						labirynt_info_array[y_rect][x_rect].e_wall = find_wall(dst_rotate, helper, 0, 1, center);
						labirynt_info_array[y_rect][x_rect].s_wall = find_wall(dst_rotate, helper, 1, 1, center);
						//to powinno byc w konstruktorze
						labirynt_info_array[y_rect][x_rect].value = -1;

						//cout<<labirynt_info_array[y_rect][x_rect].n_wall<<endl;
					}
				}

//				for(y_rect=0; y_rect < LABIRYNT_SIZE_Y; y_rect++)
//				{
//					for(x_rect=0; x_rect < LABIRYNT_SIZE_X; x_rect++)
//					{
//						std::cout<<labirynt_info_array[y_rect][x_rect].n_wall<<" "<<labirynt_info_array[y_rect][x_rect].w_wall<<" "<<labirynt_info_array[y_rect][x_rect].e_wall<<" "<<labirynt_info_array[y_rect][x_rect].s_wall<<"     ";
//					}
//					std::cout<<std::endl;
//				}

/***********************/
/** FIND START FIELD (knowing ball_center point) */
/***********************/

				int FIELD_X = ((corner_RD.x - corner_LU.x)/LABIRYNT_SIZE_X);
				int FIELD_Y = ((corner_RD.y - corner_LU.y)/LABIRYNT_SIZE_Y);

				int xfield = -1,yfield = -1;

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
				if(xfield < 0 || yfield <0)
				{
					std::cout<<"KULKA POZA LABIRYNTEM"<<endl;
					return;
				}
				else
				{
/***********************/
/** popraw jezeli przy zamalowaniu pilki zamalowalismy scianke */
/***********************/

					//pixelowo srodek pola gdzie jest pilka
					Point ctr;
					ctr.x = corner_LU.x +FIELD_X/2 + xfield*FIELD_X;
					ctr.y = corner_LU.y +FIELD_Y/2 + yfield*FIELD_Y;

					labirynt_info_array[yfield][xfield].n_wall = find_wall_ball(dst_rotate, helper, 1,-1, ctr);
					labirynt_info_array[yfield][xfield].w_wall = find_wall_ball(dst_rotate, helper, 0,-1, ctr);
					labirynt_info_array[yfield][xfield].e_wall = find_wall_ball(dst_rotate, helper, 0, 1, ctr);
					labirynt_info_array[yfield][xfield].s_wall = find_wall_ball(dst_rotate, helper, 1, 1, ctr);

/***********************/
/** FIND PATH */
/***********************/
					/* find path */
					if (find_path(xfield,yfield) == false)
					{
						printf("Labirynt nie ma drogi wyjscia!!\n");
					}
					else
					{
						printf("Znaleziono wyjscie - Oto sciezka:\n");
						for(unsigned int i=0;i<path.size();i++)
						{
							printf("%d,%d\n",path[i].x,path[i].y);
						}
					}
/***********************/
/** POPRAWA JEZELI PILKA NIE W CENTRUM POLA */
/***********************/

					if(path.size()>2)
					{
						//pixelowo srodek pola gdzie jest pilka
						int x_field_center = corner_LU.x + FIELD_X/2 + xfield*FIELD_X;
						int y_field_center = corner_LU.y + FIELD_Y/2 + yfield*FIELD_Y;

						int xdiff = ball_center.x - x_field_center;
						int ydiff = ball_center.y - y_field_center;

						int roznica = FIELD/3;
						std::cout<<"roznica "<<roznica<<" a wlasciwa: "<<xdiff<<" i "<<ydiff<<std::endl;

						if(xdiff > roznica || xdiff < -roznica || ydiff > roznica || ydiff < -roznica)
							set_difference(xdiff,ydiff);
					}
					printf("Oto poprawiona sciezka:\n");
					for(unsigned int i=0;i<path.size();i++)
					{
						printf("%d,%d\n",path[i].x,path[i].y);
					}

				}

//			}//circle.size()!=0


			has_image = true;
			out_img.write(dst_rotate);//contour / image
			newImage->raise();


		}//has_image
		else
		{
			//has_image == true
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

		//cv::line( img, cv::Point( search.x,search.y),
		//	cv::Point(search.x,search.y), 0, 1, 8 );

		//out_img.write(img);//contour / image
		//newImage->raise();

		if(pix.val[0] < 255.0)
		{
			/*wydruki*/
			//std::cout<<search.x<<" "<<search.y<<endl;
//			if(flag==0)
//			{
//				if(value==1)
//					printf("SCIANA: NA PRAWO\n");
//				else
//					printf("SCIANA: NA LEWO\n");
//			}
//			if(flag==1)
//			{
//				if(value==1)
//					printf("SCIANA: NA DOLE\n");
//				else
//					printf("SCIANA: NA GORZE\n");
//			}

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

bool CvFindLabirynth_Processor::find_wall_ball(cv::Mat img, cv::Mat dst, int flag, int value, cv::Point2i center)
{
	for(int ii=0;ii<4;ii++)
	{
		//sprawdzaj blisko naroznikow danego pola
		cv::Point2i kulka = center;
		int eps = WALL - 10;//WALL*(3/4);
		if(ii==0)
		{
			std::cout<<"1"<<std::endl;
			kulka.x = kulka.x + eps;
			kulka.y = kulka.y + eps;
		}
		else if(ii==1)
		{
			std::cout<<"2"<<std::endl;
			kulka.x -= eps;
			kulka.y += eps;
		}
		else if(ii==2)
		{
			std::cout<<"3"<<std::endl;
			kulka.x -= eps;
			kulka.y -= eps;
		}
		else if(ii==3)
		{
			std::cout<<"4"<<std::endl;
			kulka.x += eps;
			kulka.y -= eps;
		}

		std::cout<<"szukam od: "<<kulka.x<<" "<<kulka.y<<endl;
		cv::Scalar pixy;
		for(int i=1;i<(WALL);i++)
		{

			pixy.val[0] = img.at<uchar>(kulka.y, kulka.x);
			if(pixy.val[0] < 255.0)
			{
				std::cout<<"sciana w: "<<kulka.x<<" "<<kulka.y<<endl;
				return true;
			}

			if(flag == 0)
				kulka.x = kulka.x + value;
			if(flag == 1)
				kulka.y = kulka.y + value;
		}
	}
	std::cout<<"sciana - nie ma"<<endl;
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

void CvFindLabirynth_Processor::set_difference(int xdiff,int ydiff)
{
	int temp = path.size();
	Point_labirynth first, next, zero;
	first.x = path[temp-1].x;
	first.y = path[temp-1].y;
	next.x = path[temp-2].x;
	next.y = path[temp-2].y;

	if(first.x < next.x)//w prawo
	{
		if(xdiff>0 && abs(xdiff)>abs(ydiff))//x+
			return;
		if(xdiff<0 && abs(xdiff)>abs(ydiff))//x-
		{
			zero.x = first.x-1;
			zero.y = first.y;
			path.push_back(zero);
			return;
		}
		if(ydiff>0 && abs(xdiff)<abs(ydiff))//y+
		{
			zero.x = first.x;
			zero.y = first.y+1;
			path.push_back(zero);
			return;
		}
		if(ydiff<0 && abs(xdiff)<abs(ydiff))//y-
		{
			zero.x = first.x;
			zero.y = first.y-1;
			path.push_back(zero);
			return;
		}
	}
	else if(first.x > next.x)//w lewo
	{
		if(xdiff>0 && abs(xdiff)>abs(ydiff))//x+
		{
			zero.x = first.x+1;
			zero.y = first.y;
			path.push_back(zero);
			return;
		}
		if(xdiff<0 && abs(xdiff)>abs(ydiff))//x-
			return;
		if(ydiff>0 && abs(xdiff)<abs(ydiff))//y+
		{
			zero.x = first.x;
			zero.y = first.y+1;
			path.push_back(zero);
			return;
		}
		if(ydiff<0 && abs(xdiff)<abs(ydiff))//y-
		{
			zero.x = first.x;
			zero.y = first.y+1;
			path.push_back(zero);
			return;
		}
	}
	else if(first.y < next.y)//w dol
	{
		if(xdiff>0 && abs(xdiff)>abs(ydiff))//x+
		{
			zero.x = first.x+1;
			zero.y = first.y;
			path.push_back(zero);
			return;
		}
		if(xdiff<0 && abs(xdiff)>abs(ydiff))//x-
		{
			zero.x = first.x-1;
			zero.y = first.y;
			path.push_back(zero);
			return;
		}
		if(ydiff>0 && abs(xdiff)<abs(ydiff))//y+
			return;
		if(ydiff<0 && abs(xdiff)<abs(ydiff))//y-
		{
			zero.x = first.x;
			zero.y = first.y-1;
			path.push_back(zero);
			return;
		}
	}
	else if(first.y > next.y)//w gore
	{
		if(xdiff>0 && abs(xdiff)>abs(ydiff))//x+
		{
			zero.x = first.x+1;
			zero.y = first.y;
			path.push_back(zero);
			return;
		}
		if(xdiff<0 && abs(xdiff)>abs(ydiff))//x-
		{
			zero.x = first.x-1;
			zero.y = first.y;
			path.push_back(zero);
			return;
		}
		if(ydiff>0 && abs(xdiff)<abs(ydiff))//y+
		{
			zero.x = first.x;
			zero.y = first.y+1;
			path.push_back(zero);
			return;
		}
		if(ydiff<0 && abs(xdiff)<abs(ydiff))//y-
			return;
	}

}



} // namespace CvFindLabirynth {
} // namespace Processors {
