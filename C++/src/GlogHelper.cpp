#include "GlogHelper.hpp"

//配置输出日志的目录：
#define LOGDIR "log"
#define MKDIR "mkdir -p " LOGDIR

//将信息输出到单独的文件和 LOG(ERROR)
void SignalHandle(const char *data, int size)
{
    std::string str = std::string(data, size);
    /*
    std::ofstream fs("glog_dump.log",std::ios::app);
    fs<<str;
    fs.close();
    */
    LOG(ERROR) << str;
    //也可以直接在这里发送邮件或短信通知，不过这个方法是被回调多次的（每次回调只输出一行错误信息，所以如上面的记录到文件，也需要>以追加模式方可），所以这里发邮件或短信不是很适合，不过倒是可以调用一个 SHELL 或 PYTHON 脚本，而此脚本会先 sleep 3秒左右，然后将错
    //误信息通过邮件或短信发送出去，这样就不需要监控脚本定时高频率执行，浪费效率了。
}

//GLOG配置：
GLogHelper::GLogHelper(char *program)
{
    system(MKDIR);
    google::InitGoogleLogging(program);

    google::SetStderrLogging(google::ERROR);                         //设置级别高于 google::INFO 的日志同时输出到屏幕
    FLAGS_colorlogtostderr = true;                                  //设置输出到屏幕的日志显示相应颜色
    //google::SetLogDestination(google::ERROR,"log/error_");        //设置 google::ERROR 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::INFO, LOGDIR "/INFO_");       //设置 google::INFO 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::WARNING, LOGDIR "/WARNING_"); //设置 google::WARNING 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::ERROR, LOGDIR "/ERROR_");     //设置 google::ERROR 级别的日志存储路径和文件名前缀
    FLAGS_logbufsecs = 0;                                           //缓冲日志输出，默认为30秒，此处改为立即输出
    FLAGS_max_log_size = 100;                                       //最大日志大小为 100MB
    FLAGS_stop_logging_if_full_disk = true;                         //当磁盘被写满时，停止日志输出
    google::SetLogFilenameExtension("91_");                         //设置文件名扩展，如平台？或其它需要区分的信息
    google::InstallFailureSignalHandler();                          //捕捉 core dumped
    google::InstallFailureWriter(&SignalHandle);                    //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr，可以通过下面的方法自定义输出>方式：
}
//GLOG内存清理：
GLogHelper::~GLogHelper()
{
    google::ShutdownGoogleLogging();
}