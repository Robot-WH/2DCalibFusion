
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main() {
    cv::VideoCapture cap("/dev/video10");  // 打开设备号为/dev/video0的USB相机
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    cv::namedWindow("USB Camera", cv::WINDOW_NORMAL);

    while (true) {
        cap >> frame;  // 读取一帧图像
        std::cout << "get img" << std::endl;
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }

        cv::imshow("USB Camera", frame);  // 显示图像

        // 等待按下ESC键退出
        char key = cv::waitKey(10);
        if (key == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}