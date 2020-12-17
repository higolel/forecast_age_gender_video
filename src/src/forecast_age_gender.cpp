#include "forecast_age_gender.h"

// --------------------------------------------------------
/// \概要:	msleep
///
/// \参数:	image
// --------------------------------------------------------
void sleep_ms(unsigned int millisec)
{
	struct timeval tval;
	tval.tv_sec = millisec / 1000;
	tval.tv_usec = (millisec * 1000) % 1000000;
	select(0, NULL, NULL, NULL, &tval);
}

// --------------------------------------------------------
/// \概要:	获得参数
///
/// \参数:	w
/// \参数:	h
///
/// \返回:	FaceSDKConfig
// --------------------------------------------------------
FaceSDKConfig getConfig(int w, int h)
{
    FaceSDKConfig config;
    config.img_w = w;
    config.img_h = h;
    config.screen_w = w;
    config.screen_h = h;
    config.input_format = ImageFormat::BGR;
    config.mode = FaceSDKMode::Normal;
    config.thread_num = 2;
    return config;
}

// --------------------------------------------------------
/// \概要:	给图片添加文字
///
/// \参数:	image
/// \参数:	text
/// \参数:	origin
// --------------------------------------------------------
void Add_text_to_pic(cv::Mat &image, std::string text, cv::Point origin)
{
	int fontHeight = 50;
	int thickness = -1;
	int linestyle = 8;
	int baseline = 0;

	cv::Ptr<cv::freetype::FreeType2> ft2;
	ft2 = cv::freetype::createFreeType2();
	ft2->loadFontData(ch_ttf_, 0);

	ft2->putText(image, text, origin, fontHeight, cv::Scalar(0, 0, 255), thickness, linestyle, true);
}

// --------------------------------------------------------
/// \概要:	前摄像头接收线程
// --------------------------------------------------------
void Receive()
{
	cv::VideoCapture cap;
	cap.open(video_path_);
	if(!cap.isOpened())
		std::cerr << "open video failed!" << std::endl;
	else
		std::cout << "open video success!" << std::endl;

	cv::Mat frame;
	bool isSuccess = true;
	while(1)
	{
		isSuccess = cap.read(frame);
		if(!isSuccess)
		{
			std::cerr << "video ends!" << endl;
			break;
		}

		que_cv.push(frame);
		if(que_cv.size() > 1)
			// 注意 pop和front的区别
			que_cv.pop();
		else
			sleep_ms(10);
	}
}

// --------------------------------------------------------
/// \概要:	前摄像头显示线程
// --------------------------------------------------------
void Display()
{
	cv::Mat frame;
	while(1)
	{
		sleep_ms(20);
		if(!que_cv.empty())
		{
			frame = que_cv.front();
			que_cv.pop();
		}
		else
			continue;

		int w = frame.cols % 2 + frame.cols;
		int h = frame.rows % 2 + frame.rows;

		cv::resize(frame, frame, cv::Size(w, h));
		FaceSDKConfig config = getConfig(w, h);
		facesdk_init(config);

		char data[w * h * 3];
		memcpy(data, (char *)frame.data, w * h * 3);
		facesdk_readModelFromFile(ModelType::Detect, model_detect_.c_str(), ImageFormat::RGB);
		sdkFaces faces = facesdk_detect(data);
	//	std::cout << "faces:" << faces.face_count << std::endl;

		for (int i = 0; i < faces.face_count; i++)
		{
			cv::Point pt1(faces.info[i].face_box.x1, faces.info[i].face_box.y1);
			cv::Point pt2(faces.info[i].face_box.x2, faces.info[i].face_box.y2);
			cv::rectangle(frame, pt1, pt2, cv::Scalar(255, 0, 0), 2);
		}

		facesdk_readModelFromFile(ModelType::Attribution, model_attribution_.c_str(), ImageFormat::RGB);
		sdkFaces faces3 = facesdk_attribute();
		std::string text;

		for (int i = 0; i < faces3.face_count; i++)
		{
			if(faces3.info[i].attribution.gender == 0)
				text = std::string("年龄：") + std::to_string(faces3.info[i].attribution.age) + std::string(" 性别： M");
			else if(faces3.info[i].attribution.gender == 1)
				text = std::string("年龄：") + std::to_string(faces3.info[i].attribution.age) + std::string(" 性别： W");

			cv::Point origin(faces.info[i].face_box.x1, faces.info[i].face_box.y1 - 60);
			Add_text_to_pic(frame, text, origin);
	 	}
#if 1
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		pub_image_.publish(msg);
#endif
	}
}

// --------------------------------------------------------
/// \概要:	主函数
///
/// \参数:	argc
/// \参数:	argv[]
///
/// \返回:	int
// --------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "forecast_age_gender");
	ros::NodeHandle nh_("~");
	ros::Time time = ros::Time::now();
	ros::Rate loop_rate(10);

	nh_.param("/model_detect", model_detect_, std::string("./models/mtcnn_frozen_model.pb"));
	nh_.param("/model_attribution", model_attribution_, std::string("./models/mtcnn_frozen_model.pb"));
	nh_.param("/output_image", output_image_, std::string("./output_image/output_image01.jpg"));
	nh_.param("/ch_ttf", ch_ttf_, std::string("./output_image/output_image01.jpg"));
	nh_.param("video_path", video_path_, std::string("test.mp4"));
	nh_.param("cam", cam_, std::string("right_front"));

	video_path_ = video_path_ + video_sub_;

#if 1
	image_transport::ImageTransport it(nh_);
	pub_image_ = it.advertise("/camera/image_" + cam_, 1);
#endif

//	pub_plate_pic_message_ = nh_.advertise<face_plate_msgs::Plate_pic>("/plate_pic_msg", 1);

	std::thread thread_1(Receive);
	std::thread thread_2(Display);

	ros::spin();

	thread_1.join();
	thread_2.join();

	return 0 ;
}
