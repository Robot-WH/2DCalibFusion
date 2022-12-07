
#pragma once 
namespace hectorslam {
namespace util {

    /**
     * @brief:  BinderWrapper 基类  
     * @details:  在 function中只依赖这个基类 ， 对外的 函数为Call  设计为虚函数 供外界调用，BinderWrapperImpl 为它的子类 ，在 BinderWrapperImpl  中
     *                        完成对于binder 的保存以及Call的实现  
     * @param _return_type 函数返回型别
     * @param _args_type 函数参数型别 
     */    
    template<typename _return_type, typename... _args_type>
    class BinderWrapperBase {
        public:
            virtual ~BinderWrapperBase() {}
            /**
             * @brief:  调用绑定的函数 
             * @param _args_type 函数的输入参数类型
             * @return _return_type 注意 由于是 虚函数  所以这里不能用auto返回  ，所以采用模板
             */            
            virtual _return_type Call(_args_type... args) = 0;  
    };  // class BinderWrapperBase

    template<typename _binder_type, typename _return_type, typename... _args_type>
    class BinderWrapperImpl : public BinderWrapperBase<_return_type, _args_type...> {
        public:
            BinderWrapperImpl(_binder_type binder) : binder_(binder) {}
            /**
             * @brief:  调用绑定的函数    重写   
             * @details:  调用 binder_ 实现  
             * @param _args_type 函数的输入参数类型
             * @return _return_type 模板参数  
             */            
            _return_type Call(_args_type... args) override {
                return binder_(args...);
            }
        private:
            _binder_type binder_;  
    }; // class BinderWrapperImpl

    class FunctionBase {
        public:
            virtual ~FunctionBase() {
                std::cout << "~FunctionBase()" <<std::endl;
            }
    };

    /**
     * @brief:  std::Function 
     * @details: 
     * @param __Fx Function 绑定的函数类型
     */    
    template<typename _return_type, typename... _args_type>
    class Function : public FunctionBase {}; // class Function

    /**
     * @brief:  std::Function 偏特例化
     * @details: 为了支持 类似与 void(int, int, string) 这种传参方式
     * @param __Fx Function 绑定的函数类型
     */    
    template<typename _return_type, typename... _args_type>
    class Function<_return_type(_args_type...)> : public FunctionBase {
        public:
            Function() {}
            // 构造    可以传入函数指针， 也可以i是bind的 返回  
            template<typename _binder_type>
            Function(_binder_type binder) {
                binder_wrapper_ = 
                    new BinderWrapperImpl<_binder_type, _return_type, _args_type...>{binder};  
            }
            // // 拷贝赋值
            // Function<_return_type(_args_type ... )>& operator= (Function<_return_type(_args_type ... )> const& a) {
            // }
            // 析构函数  
            virtual ~Function() {
                std::cout << "~Function()" <<std::endl;
                delete binder_wrapper_;   
            }
            // 仿函数  
            _return_type operator()(_args_type... args) {
                return binder_wrapper_->Call(args...); 
            }
        private:
            BinderWrapperBase<_return_type, _args_type...>* binder_wrapper_ = nullptr;  
    }; // class Function
}; 
}