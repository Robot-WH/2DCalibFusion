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
        void test_int(int data) {
            std::cout << "test  int callback, data: " <<data <<std::endl;
        }
        void test_int_fast(int data) {
            std::cout << "test  int callback fast, data: " <<data <<std::endl;
        }

        void test_int_ref(int& data) {
            std::cout << "test_int_ref callback, data: " <<data <<std::endl;
        }
        void test_int_ref_fast(int& data) {
            std::cout << "test_int_ref callback fast, data: " <<data <<std::endl;
        }

        void test_const_int_ref(const int& data) {
            std::cout << "test_const_int_ref callback, data: " <<data <<std::endl;
        }
        void test_const_int_ref_fast(const int& data) {
            std::cout << "test_const_int_ref callback fast, data: " <<data <<std::endl;
        }

        void test_int_right_ref(int&& data) {
            std::cout << "test_int_right_ref callback, data: " <<data <<std::endl;
        }
        void test_int_right_ref_fast(int&& data) {
            std::cout << "test_int_right_ref callback fast, data: " <<data <<std::endl;
        }

};

class Read {
    public:
        int& get_value_ref() {
            return value;
        }
        const int& get_value_const_ref() {
            return value;
        }
    private:
        int value = 100;
} read;


int main() {
    Test test; 
    // 订阅数据容器(topic), 回调函数，5是缓存大小
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("int_data", &Test::test_int_fast, &test, 5, 1);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("int_data", &Test::test_int, &test, 5, 0);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("test_int_ref", &Test::test_int_ref_fast, &test, 5, 1);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("test_int_ref", &Test::test_int_ref, &test, 5, 0);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("test_const_int_ref", &Test::test_const_int_ref_fast, &test, 5, 1);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("test_const_int_ref", &Test::test_const_int_ref, &test, 5, 0);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("test_int_right_ref", &Test::test_int_right_ref_fast, &test, 5, 1);
    hectorslam::util::DataDispatcher::GetInstance().Subscribe("test_int_right_ref", &Test::test_int_right_ref, &test, 5, 0);
    
    // 1、测试数据容器
    // 证明 typeid() 不区分const 和 & 
    std::type_index type_info(typeid(const float&));  // 数据类型信息 
    std::type_index type_info2(typeid(float));  // 数据类型信息 
    if (type_info == type_info2) {
        std::cout << "type_info == type_info2" << std::endl;
    }

    int value = 99; 
    /************************************************************* 一下测试全部通过 *****************************************/
    // 测试回调函数是值传递
    hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", 10);  // 发送右值
    hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", value);  // 发送左值
    hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", read.get_value_ref());  // 发送引用  
    hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", read.get_value_const_ref());  // 发送常量引用
    hectorslam::util::DataDispatcher::GetInstance().Publish("int_data", std::move(value));  // 发送右值引用

    // // 测试回调函数是引用传递 
    // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_ref", value);  // 发送左值   对于非高优先级非法
    // // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_ref", 12);  // 发送右值    非法操作  
    // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_ref", read.get_value_ref());  // 发送引用  
    // // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_ref", read.get_value_const_ref());  // 发送常量引用   非法操作  
    // // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_ref", std::move(value));  // 发送右值引用  非法

    // 测试回调是常量引用
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_const_int_ref", value);  // 发送左值
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_const_int_ref", 12);  // 发送右值   
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_const_int_ref", read.get_value_ref());  // 发送引用  
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_const_int_ref", read.get_value_const_ref());  
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_const_int_ref", std::move(value));  // 发送右值引用

    // // 测试回调是右值引用
    // // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_right_ref", value);  // 发送左值      非法
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_right_ref", 17);  // 发送右值
    hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_right_ref", std::move(value));  // 发送右值引用
    // // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_right_ref", read.get_value_ref());  // 发送左值引用  非法
    // // hectorslam::util::DataDispatcher::GetInstance().Publish("test_int_right_ref", read.get_value_const_ref());   // 发送常量左值引用  非法
    /************************************************************************************************************************/
    hectorslam::util::DataDispatcher::GetInstance().GetThread().join(); 
    return 1; 
}