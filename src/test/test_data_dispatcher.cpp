/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-11-17 20:31:27
 * @Description: 
 * @Others: 
 */

#include "../FrontEnd2D/include/util/DataDispatcher.hpp"

class Test {
    public:
        void test_int(const int& data) {
            std::cout << "test  int callback, data: " <<data <<std::endl;
        }

        void test_string(const std::string& data) {
            std::cout << "test  string callback, data: " <<data <<std::endl;
        }
};

void thread_one() {
    while(1) {
        hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void thread_two() {
    while(1) {
        hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void thread_three() {
    while(1) {
        hectorslam::util::DataDispatcher::GetInstance().Publish("string_data", std::string("hello"));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



int main() {
    Test test; 
    // 订阅数据容器(topic), 回调函数，5是缓存大小
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("int_data", &Test::test_int, &test, 5);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("string_data", &Test::test_string, &test, 5);
    // 1、测试数据容器
    // 证明 typeid() 不区分const 和 & 
    std::type_index type_info(typeid(const float&));  // 数据类型信息 
    std::type_index type_info2(typeid(float));  // 数据类型信息 
    if (type_info == type_info2) {
        std::cout << "type_info == type_info2" << std::endl;
    }

    std::thread th1(&thread_one);
    std::thread th2(&thread_two);
    std::thread th3(&thread_three);

    th1.join();
    th2.join();
    th3.join();

    return 1; 
}