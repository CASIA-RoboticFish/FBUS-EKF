#ifndef GLOG_HELPER_HPP
#define GLOG_HELPER_HPP
#include <stdlib.h>
#include <glog/logging.h>
#include <glog/raw_logging.h>
// #include "IMSEE-SDK/include/logging.h" // This is google logging

//将信息输出到单独的文件和 LOG(ERROR)
void SignalHandle(const char* data, int size);

class GLogHelper
{
public:
    //GLOG配置：
    GLogHelper(char* program);
    //GLOG内存清理：
    ~GLogHelper();
};

#endif