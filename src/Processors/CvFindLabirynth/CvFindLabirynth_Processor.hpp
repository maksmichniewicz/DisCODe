
#ifndef CVFINDLABIRYNTH_PROCESSOR_HPP_
#define CVFINDLABIRYNTH_PROCESSOR_HPP_

#include <string.h>
#include <cv.h>
#include <boost/shared_ptr.hpp>
#include "Component_Aux.hpp"
#include "Panel_Empty.hpp"
#include "Objects3D/Chessboard.hpp"
#include "ImagePosition.hpp"

#include "LReading.hpp"

#include "xdr/xdr_iarchive.hpp"

//#include "Proxies/Mrrocpp/Mrrocpp_Proxy.hpp"
//#include "Proxies/Mrrocpp/headers.h"

#include "Drawable.hpp"
#include "Timer.hpp"

#include "Property.hpp"

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "Panel_Empty.hpp"
#include "DataStream.hpp"
#include "Props.hpp"

#include <cv.h>
//using namespace cv;
#include <highgui.h>

#include <vector>

#define LABIRYNT_SIZE_X 7
#define LABIRYNT_SIZE_Y 7

/**
 * info about walls for one rectangle in Labirynth
 * */
struct walls_info
{
	bool n_wall;
	bool e_wall;
	bool w_wall;
	bool s_wall;
	int value;
};
/**
 * Point struct
 * */
struct Point_labirynth
{
	int x;
	int y;
};

namespace Processors {
namespace CvFindLabirynth {

class CvFindLabirynth_Processor: public Base::Component
{
public:
	CvFindLabirynth_Processor(const std::string & name = "");
	virtual ~CvFindLabirynth_Processor();

	int EPS;
	int FIELD;
	int WALL;

	int TEMP;
	bool HELP;

protected:
	/*!
	 * Method called when component is started
	 * \return true on success
	 */
	virtual bool onStart();

	/*!
	 * Method called when component is stopped
	 * \return true on success
	 */
	virtual bool onStop();

	/*!
	 * Method called when component is initialized
	 * \return true on success
	 */
	virtual bool onInit();

	/*!
	 * Method called when component is finished
	 * \return true on success
	 */
	virtual bool onFinish();

	/*!
	 * Method called when step is called
	 * \return true on success
	 */
	virtual bool onStep();


	/**
	* find labirynth's walls
	*/
	bool find_labirynth(cv::Mat img, cv::Mat dst, int flag, int value);

	/**
	* find walls inside labirynth
	*/
	bool find_wall(cv::Mat img, cv::Mat dst, int flag, int value, cv::Point2i center);
	/**
	* sprawdz czy zamalowanie pilki nie zamalowalo scianki i popraw to.
	*/
	bool find_wall_ball(cv::Mat img, cv::Mat dst, int flag, int value, cv::Point2i center);

	/**
	* find optimal path
	*/
	bool find_path(int=0,int=0);
	/**
	* helper method for find_path
	*/
	void set_neighbours(int y, int x);

	/**
	* helper method for find_path
	*/
	void set_path(Point_labirynth end, Point_labirynth start);

	void set_difference(int xdiff,int ydiff);





	walls_info labirynt_info_array [LABIRYNT_SIZE_Y][LABIRYNT_SIZE_X];
	int q;
	bool path_exists;
	bool path_set;
	vector<Point_labirynth> path;

	cv::Point2i corner_LU;
	cv::Point2i corner_RD;

	cv::Point2i ball_center;


private:
	void onNewImage();
	void onRpcCall();

	/** event handler. */
	Base::EventHandler <CvFindLabirynth_Processor> h_onNewImage;
	Base::EventHandler <CvFindLabirynth_Processor> h_onRpcCall;
	/** In stream. */
	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_img;
	Base::DataStreamIn <xdr_iarchive <> > rpcParam;
	/** out event and stream */
	Base::Event* rpcResult;
	Base::DataStreamOut <Types::Mrrocpp_Proxy::LReading> out_info;
	//changed image
	Base::Event * newImage;
	Base::DataStreamOut <cv::Mat> out_img;

	/** Located corners.*/
	std::vector<cv::Point2f> corners;
	int findLabirynthFlags;
	int temp;
	bool found;

	cv::Mat image;
	bool has_image;

};

} // namespace CvFindLabirynth {

} // namespace Processors {

REGISTER_PROCESSOR_COMPONENT("CvFindLabirynth", Processors::CvFindLabirynth::CvFindLabirynth_Processor, Common::Panel_Empty)

#endif /* CVFINDLABIRYNTH_PROCESSOR_HPP_ */
