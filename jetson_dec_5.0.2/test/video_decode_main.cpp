#include "JetsonDec.h"
#include "h264.h"
#if 0
// H264 NALU 裸流测试
class Wrapper : public JetsonDecListner
{
public:
    Wrapper(char *path);
    void OnJetsonDecData(unsigned char *data, int data_len, uint64_t timestamp);

public:
    JetsonDec *jetson_dec_obj_ = NULL;
    std::string file_;
    uint32_t decoder_pixfmt_;
    unsigned char jetson_addr_[3 * 2000 * 2000];
    int64_t frames_ = 0;
};
Wrapper::Wrapper(char *path)
{
    file_ = path;
    jetson_dec_obj_ = new JetsonDec(V4L2_PIX_FMT_H264, 1920, 1080, jetson_addr_);
    decoder_pixfmt_ = V4L2_PIX_FMT_H264;
    jetson_dec_obj_->SetDecCallBack(static_cast<JetsonDecListner *>(this));
}
char *dec_filename = "out.yuv";
FILE *dec_fd = NULL;
void Wrapper::OnJetsonDecData(unsigned char *data, int data_len, uint64_t timestamp)
{
    frames_++;
    struct timeval time_dec;
    gettimeofday(&time_dec, NULL);
    long int time_stamp = 1000 * (time_dec.tv_sec) + (time_dec.tv_usec) / 1000;
    if (frames_ % 20 == 0) {
        printf("delta:%ld\n", time_stamp - timestamp);
    }
#if 0
    if(dec_fd==NULL){
            dec_fd= fopen(dec_filename, "wb");
    }
    fwrite(jetson_addr_, 1, 1920 * 1080 * 3/2, dec_fd);
#endif
}
void OnData(char *pBuf, int len, void *param)
{
    Wrapper *self = (Wrapper *)param;
    int try_cnt = 0;
    while (self->jetson_dec_obj_->GetQueueSize() > 5 && try_cnt < 4) {
        usleep(1000 * 10);
        try_cnt++;
        printf("GetQueueSize:%d\n", self->jetson_dec_obj_->GetQueueSize());
    }
    self->jetson_dec_obj_->AddEsData((unsigned char *)pBuf, len);
    return;
}
int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("./demo file stress_num\n");
        return -1;
    }
    int stress = atoi(argv[2]);
    printf("stress:%d\n", stress);
    while (stress > 0) {
        Wrapper *test = new Wrapper(argv[1]);
        NALUInit(argv[1], OnData, test);
        stress--;
    }
    while (true) {
        usleep(1000 * 1000 * 10);
    }

    return 0;
}
#else
// MP4文件测试
#include "MediaInterface.h"
#include "MediaReader.h"
#include <fstream>
#include <iostream>
#include <vector>
class Wrapper : public JetsonDecListner, public MediaDataListner
{
public:
    Wrapper(char *path);
    ~Wrapper();
    void OnJetsonDecData(unsigned char *data, int data_len, uint64_t timestamp);
    void OnVideoData(VideoData data);
    void OnAudioData(AudioData data);
    void MediaOverhandle();
public:
    MediaReader *file_reader_;
    JetsonDec *jetson_dec_obj_ = NULL;
    std::string file_;
    uint32_t decoder_pixfmt_;
    unsigned char *jetson_addr_ = NULL;
    int64_t frames_ = 0;
    int width_, height_;
    uint64_t total_ = 0;
};
Wrapper::Wrapper(char *path)
{
    file_reader_ = new MediaReader(path);
    file_reader_->setDataListner(static_cast<MediaDataListner *>(this), [this]() {
        return this->MediaOverhandle();
    });
    file_ = path;
}
Wrapper::~Wrapper()
{
    if (file_reader_) {
        delete file_reader_;
        file_reader_ = NULL;
    }
    if (jetson_dec_obj_) {
        delete jetson_dec_obj_;
        jetson_dec_obj_ = NULL;
    }
    if (jetson_addr_) {
        free(jetson_addr_);
        jetson_addr_ = NULL;
    }
}
// with startcode
void Wrapper::OnVideoData(VideoData data)
{
    enum VideoType videoType = file_reader_->getVideoType();
    width_ = file_reader_->m_pFormatCtx->streams[file_reader_->video_index]->codecpar->width;
    height_ = file_reader_->m_pFormatCtx->streams[file_reader_->video_index]->codecpar->height;
    if (videoType == VIDEO_H264) {
        decoder_pixfmt_ = V4L2_PIX_FMT_H264;
    }
    if (videoType == VIDEO_H265) {
        decoder_pixfmt_ = V4L2_PIX_FMT_H265;
    }
    if (jetson_dec_obj_ == NULL) {
        jetson_addr_ = (unsigned char *)malloc(width_ * height_ * 4);
        printf("width:%d height:%d\n", width_, height_);
        jetson_dec_obj_ = new JetsonDec(decoder_pixfmt_, width_, height_, jetson_addr_);
        jetson_dec_obj_->SetDecCallBack(static_cast<JetsonDecListner *>(this));
    }
    int try_cnt = 0;
    while (jetson_dec_obj_->GetQueueSize() > 5 && try_cnt < 4) {
        usleep(1000 * 10);
        try_cnt++;
        printf("GetQueueSize:%d\n", jetson_dec_obj_->GetQueueSize());
    }
    jetson_dec_obj_->AddEsData((unsigned char *)data.data, data.data_len);
    return;
}
void Wrapper::OnAudioData(AudioData data)
{
    printf("OnAudioData\n");
    return;
}
void Wrapper::MediaOverhandle()
{
    printf("MediaOverhandle....\n");
    file_reader_->reset();
    return;
}
char *dec_filename = "out.yuv";
FILE *dec_fd = NULL;
void Wrapper::OnJetsonDecData(unsigned char *data, int data_len, uint64_t timestamp)
{
    struct timeval time_dec;
    gettimeofday(&time_dec, NULL);
    long int time_stamp = 1000 * (time_dec.tv_sec) + (time_dec.tv_usec) / 1000;

    frames_ ++;
    int n = 100;
    int delay = time_stamp - timestamp;
    if (frames_ > n) {
        total_ += time_stamp - timestamp;
    }
    if (frames_ % 20 == 0) {
        printf("delay:%ld avg:%d\n", delay, total_ / (frames_ - n));
        
    }
#if 0
    // write to file , NV12
    if(dec_fd==NULL){
            dec_fd= fopen(dec_filename, "wb");
    }
    fwrite(jetson_addr_, 1, width_ * height_ * 3/2, dec_fd);
#endif
}
int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("./demo file stress_num\n");
        return -1;
    }
    int num = atoi(argv[2]);
    std::vector<Wrapper *> obj_list;
    while (num-- > 0) {
        Wrapper *test = new Wrapper(argv[1]);
        obj_list.push_back(test);
    }
    while (true) {
        usleep(1000 * 1000 * 30);
    }
    for (int i = 0; i < obj_list.size(); i++) {
        Wrapper *obj = obj_list[i];
        delete obj;
    }
    return 0;
}
#endif
