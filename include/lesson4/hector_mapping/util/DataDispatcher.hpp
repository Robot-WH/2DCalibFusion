
#pragma once 
#include <set>
#include <list>
#include <deque>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <condition_variable>
#include <typeindex>
#include <type_traits>
#include <atomic>
#include "RemoveReference.hpp"
#include "Function.hpp"
namespace hectorslam {
namespace util {

/**
 * @brief: 数据容器对外接口类  
 * @details:  非模板的万能类  
 */    
class DataContainerBase {
    public:
        DataContainerBase(uint16_t const& capacity) : capacity_(capacity) {}
        virtual ~DataContainerBase() {
            // std::cout << "~DataContainerBase()" <<std::endl;
        }
        virtual inline std::type_index GetDataType() const = 0; 
        virtual inline uint16_t GetDataSize() const = 0; 
        virtual inline void DeleteFrontData() = 0; 
        virtual bool Callback() = 0;  
        virtual inline bool IsEmpty() const = 0; 
        inline uint16_t GetCapacity() const {
            return capacity_;  
        }
    protected:
        uint16_t capacity_;
};

/**
 * @brief: 实际存储数据的类   保存的类型由模板参数指定  
 * @details 负责数据的管理以及回调函数的调用
 * @param _DataT 型别要保证是非const 非&的  
 */    
template<typename _DataT>
class DataContainerImpl : public DataContainerBase {
    public:
        DataContainerImpl(uint16_t const& capacity, FunctionBase* p_callback_wrapper) 
        : DataContainerBase(capacity), 
        p_callback_wrapper_(p_callback_wrapper), 
        type_info_(typeid(_DataT)) {}    // typeid 不区分const和&  也就是 const int& 和 int 是一样的
        
        ~DataContainerImpl() {
            // std::cout << "~DataContainerImpl()" <<std::endl;
            delete p_callback_wrapper_;  
        }

        /**
         * @brief: 执行回调函数
         * @return 执行回调是否成功
         */        
        virtual bool Callback() override {
            if (data_buffer_.empty()) return false;   
            call(data_buffer_.front());   // 传入左值   
            DeleteFrontData();  
            return true;  
        }

        /**
         * @brief: 在队列最末尾添加数据 
         */             
        template<typename _DataType>
        bool AddData(_DataType&& data) {
            // 检查类型是否一致
            if (type_info_ != std::type_index(typeid(_DataType))) {
                throw std::bad_cast();  
            }
            data_m_.lock();  
            if (data_buffer_.size() >= capacity_) {
                data_buffer_.pop_front();   
            }
            data_buffer_.push_back(std::forward<_DataType>(data));
            // std::cout << "data_buffer_ add :" << data << std::endl;
            data_m_.unlock();  
            return true;  
        }
        
        /**
         * @brief: 读取数据
         * @param[out] data 读取结果
         * @param[in] index  位于队列上的序号
         */            
        // inline _DataT GetData(const int& index) {
        //     // std::cout << " index : " << index << std::endl;
        //     // std::cout << "GetData" << data_buffer_[index] << std::endl;
        //     return data_buffer_[index];  
        // }

        /**
         * @brief: 删除头数据
         */            
        inline void DeleteFrontData() override {
            data_m_.lock();
            data_buffer_.pop_front();   
            data_m_.unlock();  
        }

        virtual inline uint16_t GetDataSize() const override {
            return data_buffer_.size();  
        }

        // 获取数据类型  
        inline std::type_index GetDataType() const override {
            return type_info_;  
        }
        
        virtual inline bool IsEmpty() const override {
            return data_buffer_.size() == 0;   
        }

    private:
        /**
         * @brief: 触发回调的接口
         * @param data 万能引用  
         */        
        template<typename _T>
        void call(_T&& data) {
            Function<void(remove_reference_t<_DataT>&)>* p_callback =  
                dynamic_cast<Function<void(remove_reference_t<_DataT>&)>*>(p_callback_wrapper_);
            (*p_callback)(data);   // 由于 p_callback 绑定的参数为左值引用，所以不需要对data进行完美转发。输入为右值时，data也是个左值
        }

        std::mutex data_m_;   // 加了mutex后变成了只移型别   (禁止了拷贝构造)
        std::deque<_DataT> data_buffer_;       // 容器的型别不支持 const/&，_DataT 必须是非引用和非const的  
        std::type_index type_info_;  // 数据类型信息 
        FunctionBase* p_callback_wrapper_; 
};

/**
 * @brief: 订阅者类   
 * @details 包含订阅回调函数和订阅数据缓存
 */    
class Subscriber {
    public:
        Subscriber() : data_cache_(nullptr) {}
        /**
         * @brief: 构造函数
         * @param callback 类内成员函数地址
         * @param class_addr 类对象地址
         * @param  cache_capacity 缓存容量  
         */        
        template<typename _DataT, typename _Ctype>
        Subscriber(void (_Ctype::*callback)(_DataT), _Ctype* class_addr, int cache_capacity = 1) {
            // std::cout << "construct Subscriber" <<std::endl;
            // 去除类别的const和引用
            using pure_type = typename std::remove_const<remove_reference_t<_DataT>>::type; 
            data_cache_ = new DataContainerImpl<pure_type>(cache_capacity, 
                new Function<void(pure_type&)>(std::bind(callback, class_addr, std::placeholders::_1)));
        }

        /**
         * @brief: 构造函数
         * @param callback 普通成员函数地址
         * @param  cache_capacity 缓存容量  
         */      
        template<typename _DataT>
        Subscriber(void (*callback)(_DataT), int cache_capacity = 1) {
            using pure_type = typename std::remove_const<remove_reference_t<_DataT>>::type; 
            data_cache_ = new DataContainerImpl<pure_type>(cache_capacity, 
                new Function<void(pure_type&)>(callback));
        }

        virtual ~Subscriber() {
            // std::cout << "~Subscriber" <<std::endl;
            delete data_cache_; 
        }

        /**
         * @brief: 读取缓存区的数据 
         * @details 直接读取缓存区 最后的数据  然后将最后的数据丢弃 
         * @return 是否读取到新的数据 
         */        
        // template<typename _DataT>
        // bool ReadData(_DataT& data) {
        //     DataContainerImpl<_DataT>* data_container_ptr =  
        //         dynamic_cast<DataContainerImpl<_DataT>*>(data_cache_);
        //     if (data_container_ptr->GetDataSize() == 0) return false; 
        //     data = std::move(data_container_ptr->GetData(0));
        //     data_container_ptr->DeleteFrontData();
        //     // 直接返回
        //     return true;  
        // }

    private:
        /**
         * @brief: 回调
         */        
        bool callback() {
            return data_cache_->Callback();  
        }

        /**
         * @brief: 检查是否所有缓存数据都处理完毕
         */        
        bool cacheDataEmpty() {
            return data_cache_->IsEmpty();  
        }

        template<typename _DataT>
        void addData(_DataT&& data) {
            // 判断输入数据与数据容器的类型是否一致     const和& 不会影响  type_index的结果  
            if (data_cache_->GetDataType() != std::type_index(typeid(_DataT))) {
                std::cerr<<"DataDispatcher addData() Type ERROR !!!"<<std::endl;
                throw std::bad_cast();  
            }
            using pure_type = typename std::remove_const<remove_reference_t<_DataT>>::type; 
            DataContainerImpl<pure_type>* data_container_ptr =  
                dynamic_cast<DataContainerImpl<pure_type>*>(data_cache_);
            data_container_ptr->AddData(std::forward<_DataT>(data));     // 线程安全的
        }

        friend class DataDispatcher; 
        DataContainerBase* data_cache_;   // 数据缓存 
};

/**
 * @brief: 数据调度器 
 * @details:  负责系统各种类型数据的保存，读取   
 */    
class DataDispatcher {
    public:
        /**
         * @brief: 单例的创建函数  
         */            
        static DataDispatcher& GetInstance() {
            static DataDispatcher data_dispatcher;
            return data_dispatcher; 
        }

        virtual ~DataDispatcher() {
            for (const auto& name_set : subscriber_container_) {
                for (const auto& pt : name_set.second) {
                    delete pt;  
                }
            }
        }
        
        /**
         * @brief: 订阅某个数据容器，回调函数为类内成员的重载 
         * @details: 订阅动作，告知DataDispatcher，_Ctype类对象的callback函数要订阅名字为name的数据容器
         * @param name 数据容器名
         * @param callback 注册的回调函数
         * @param class_addr 类对象地址
         * @param cache_capacity 缓存容量 
         */        
        template<typename _DataT, typename _Ctype>
        Subscriber& Subscribe(std::string const& name, 
                                                    void (_Ctype::*callback)(_DataT), 
                                                    _Ctype* class_addr,
                                                    int cache_capacity = 1) {
            Subscriber* p_subscriber = new Subscriber(callback, class_addr, cache_capacity);
            substriber_container_m_.lock();
            active_data_container_m_.lock();  
            active_data_container_[name] = false;  
            subscriber_container_[name].insert(p_subscriber); 
            active_data_container_m_.unlock();  
            substriber_container_m_.unlock();  
            return *p_subscriber;  
        }

        /**
         * @brief: 订阅某个数据容器，回调函数为普通函数 
         * @param name 数据容器名
         * @param callback 注册的回调函数
         * @param cache_capacity 缓存容量 
         */        
        template<typename _DataT, typename _Ctype>
        Subscriber& Subscribe(std::string const& name, 
                                                        void (*callback)(_DataT), 
                                                        int cache_capacity = 1) {
            Subscriber* p_subscriber = new Subscriber(callback, cache_capacity);
            substriber_container_m_.lock();
            active_data_container_m_.lock();  
            active_data_container_[name] = false;  
            subscriber_container_[name].insert(p_subscriber); 
            active_data_container_m_.unlock();  
            substriber_container_m_.unlock();  
            return *p_subscriber;  
        }

        /**
         * @brief: 发布数据   
         * @details 向数据容器发布数据 done 
         * @param[in] name 数据的标识名
         * @param[in] data 加入的数据 
         */            
        template<typename _T>
        void Publish(std::string const& name, _T&& data) {
            // 如果有订阅者  则将数据传送到各个订阅者的数据缓存区
            std::shared_lock<std::shared_mutex> m_l(substriber_container_m_);  // 禁止subscriber_container_ 写数据
            if (subscriber_container_[name].size()) {
                ///////////////////////////////////////////////////
                data_m_.lock();  
                for (const auto& pt : subscriber_container_[name]) {
                    pt->addData(std::forward<_T>(data));     // addData是线程安全的
                }
                data_m_.unlock();
                ///////////////////////////////////////////////////
                m_l.unlock();     // = unlock_shared(), 可以对 subscriber_container_ 进行更新了
                ///////////////////////////////////////////////////
                active_data_container_m_.lock_shared();
                if (!active_data_container_.at(name)) {
                    active_data_container_.at(name) = true;    // 将name 设置为活跃 ， 表示有新数据 
                    ++active_num;
                }
                active_data_container_m_.unlock_shared();  
                ///////////////////////////////////////////////////
                // std::cout << "active_data_container_ name: " << name << "=true" << std::endl;
                con_.notify_one();  
            }
            return;
        }

    protected:
        DataDispatcher() {
            callback_thread_ = std::thread(&DataDispatcher::process, this);
        }
        DataDispatcher(DataDispatcher const& object) {}
        DataDispatcher(DataDispatcher&& object) {}

        // 处理线程   
        void process() {
            while (true) {
                std::unique_lock<std::mutex> lock_m(process_m_);
                con_.wait(lock_m,[&] {
                    return active_num > 0;
                });
                // 遍历每一个topic，并找到活跃的(接受到了新数据)
                std::shared_lock<std::shared_mutex> s_l(active_data_container_m_); // 禁止写，防止active_data_container_遍历失败
                for (const auto& obj : active_data_container_) {
                    if (!obj.second) continue;   // 如果没有新数据 则跳过 
                    // 该容器的全部订阅者都执行回调函数
                    for (const auto& pt : subscriber_container_[obj.first]) {
                        pt->callback();   // callback() 是线程安全的
                    }
                    // 检查该数据容器的全部订阅者是否都处理完了自己的数据
                    data_m_.lock(); 
                    int finish = true; 
                    for (const auto& pt : subscriber_container_[obj.first]) {
                        if (!pt->cacheDataEmpty()) {
                            finish = false;
                            break;
                        }
                    }
                    // 如果所有订阅者的数据都回调完毕  则失能
                    if (finish) {
                        active_data_container_.at(obj.first) = false;  
                        --active_num; 
                        // std::cout << "active_data_container_ erase: " << obj.first <<std::endl;
                        // std::cout << "----------------------------------------------" <<std::endl;
                    }
                    data_m_.unlock();  
                }
            };
        }

    private:
        std::mutex data_m_; 
        std::mutex process_m_; 
        std::mutex data_container_m_; 
        std::shared_mutex substriber_container_m_; 
        std::shared_mutex active_data_container_m_;  
        std::atomic<int> active_num{0}; 
        std::condition_variable con_;   
        std::thread callback_thread_;   
        std::unordered_map<std::string, std::set<Subscriber*>> subscriber_container_;    // 保存订阅话题 name 的全部 订阅者 
        std::unordered_map<std::string, bool> active_data_container_; 
    }; // class 

} // namespace 
}